#include "BlpLoader.h"

#include <QFile>
#include <QByteArray>
#include <QHash>
#include <QMutex>
#include <QMutexLocker>
#include <QtGlobal>

namespace
{
    struct Reader
    {
        const unsigned char* data = nullptr;
        qsizetype size = 0;
        qsizetype pos = 0;

        bool canRead(qsizetype n) const { return pos + n <= size; }

        bool readU8(quint8& v)
        {
            if (!canRead(1)) return false;
            v = data[pos];
            pos += 1;
            return true;
        }

        bool readU32(quint32& v)
        {
            if (!canRead(4)) return false;
            // little-endian
            v = (quint32)data[pos] |
                ((quint32)data[pos + 1] << 8) |
                ((quint32)data[pos + 2] << 16) |
                ((quint32)data[pos + 3] << 24);
            pos += 4;
            return true;
        }

        bool readBytes(void* out, qsizetype n)
        {
            if (!canRead(n)) return false;
            memcpy(out, data + pos, size_t(n));
            pos += n;
            return true;
        }

        bool seek(qsizetype newPos)
        {
            if (newPos < 0 || newPos > size) return false;
            pos = newPos;
            return true;
        }
    };

    static void setErr(QString* outError, const QString& msg)
    {
        if (outError) *outError = msg;
    }

    static quint8 alphaForPixel(const quint8* alphaData, quint32 alphaBits, quint32 pixelIndex)
    {
        if (alphaBits == 0 || !alphaData) return 255;
        if (alphaBits == 8)
        {
            return alphaData[pixelIndex];
        }
        if (alphaBits == 1)
        {
            const quint32 byteIndex = pixelIndex >> 3;
            const quint32 bitIndex = pixelIndex & 7;
            const quint8 b = alphaData[byteIndex];
            const quint8 bit = (b >> bitIndex) & 1;
            return bit ? 255 : 0;
        }
        if (alphaBits == 4)
        {
            const quint32 byteIndex = pixelIndex >> 1;
            const bool high = (pixelIndex & 1) != 0;
            const quint8 b = alphaData[byteIndex];
            const quint8 nib = high ? ((b >> 4) & 0x0F) : (b & 0x0F);
            return quint8(nib * 17); // 0..15 -> 0..255
        }
        // Unknown alpha depth
        return 255;
    }

    static void color565(quint16 c, quint8& r, quint8& g, quint8& b)
    {
        r = quint8(((c >> 11) & 31) * 255 / 31);
        g = quint8(((c >> 5) & 63) * 255 / 63);
        b = quint8((c & 31) * 255 / 31);
    }

    static void decodeDxt1(const quint8* src, quint32 w, quint32 h, std::vector<quint8>& out)
    {
        out.assign(size_t(w) * size_t(h) * 4, 0);
        const quint32 blocksX = (w + 3) / 4;
        const quint32 blocksY = (h + 3) / 4;

        for (quint32 by = 0; by < blocksY; ++by)
        {
            for (quint32 bx = 0; bx < blocksX; ++bx)
            {
                const quint8* block = src + (by * blocksX + bx) * 8;
                const quint16 c0 = quint16(block[0] | (block[1] << 8));
                const quint16 c1 = quint16(block[2] | (block[3] << 8));
                quint8 r0=0,g0=0,b0=0,r1=0,g1=0,b1=0;
                color565(c0, r0, g0, b0);
                color565(c1, r1, g1, b1);

                quint8 colors[4][4];
                colors[0][0]=r0; colors[0][1]=g0; colors[0][2]=b0; colors[0][3]=255;
                colors[1][0]=r1; colors[1][1]=g1; colors[1][2]=b1; colors[1][3]=255;

                if (c0 > c1)
                {
                    colors[2][0] = quint8((2 * r0 + r1) / 3);
                    colors[2][1] = quint8((2 * g0 + g1) / 3);
                    colors[2][2] = quint8((2 * b0 + b1) / 3);
                    colors[2][3] = 255;
                    colors[3][0] = quint8((r0 + 2 * r1) / 3);
                    colors[3][1] = quint8((g0 + 2 * g1) / 3);
                    colors[3][2] = quint8((b0 + 2 * b1) / 3);
                    colors[3][3] = 255;
                }
                else
                {
                    colors[2][0] = quint8((r0 + r1) / 2);
                    colors[2][1] = quint8((g0 + g1) / 2);
                    colors[2][2] = quint8((b0 + b1) / 2);
                    colors[2][3] = 255;
                    colors[3][0] = 0; colors[3][1] = 0; colors[3][2] = 0; colors[3][3] = 0;
                }

                quint32 code = block[4] | (block[5] << 8) | (block[6] << 16) | (block[7] << 24);
                for (quint32 py = 0; py < 4; ++py)
                {
                    for (quint32 px = 0; px < 4; ++px)
                    {
                        const quint32 idx = code & 0x3;
                        code >>= 2;
                        const quint32 x = bx * 4 + px;
                        const quint32 y = by * 4 + py;
                        if (x >= w || y >= h) continue;
                        const size_t dst = (size_t(y) * size_t(w) + x) * 4;
                        out[dst + 0] = colors[idx][0];
                        out[dst + 1] = colors[idx][1];
                        out[dst + 2] = colors[idx][2];
                        out[dst + 3] = colors[idx][3];
                    }
                }
            }
        }
    }

    static void decodeDxt3(const quint8* src, quint32 w, quint32 h, std::vector<quint8>& out)
    {
        out.assign(size_t(w) * size_t(h) * 4, 0);
        const quint32 blocksX = (w + 3) / 4;
        const quint32 blocksY = (h + 3) / 4;

        for (quint32 by = 0; by < blocksY; ++by)
        {
            for (quint32 bx = 0; bx < blocksX; ++bx)
            {
                const quint8* block = src + (by * blocksX + bx) * 16;
                const quint8* alpha = block;
                const quint8* color = block + 8;

                const quint16 c0 = quint16(color[0] | (color[1] << 8));
                const quint16 c1 = quint16(color[2] | (color[3] << 8));
                quint8 r0=0,g0=0,b0=0,r1=0,g1=0,b1=0;
                color565(c0, r0, g0, b0);
                color565(c1, r1, g1, b1);

                quint8 colors[4][4];
                colors[0][0]=r0; colors[0][1]=g0; colors[0][2]=b0; colors[0][3]=255;
                colors[1][0]=r1; colors[1][1]=g1; colors[1][2]=b1; colors[1][3]=255;
                colors[2][0] = quint8((2 * r0 + r1) / 3);
                colors[2][1] = quint8((2 * g0 + g1) / 3);
                colors[2][2] = quint8((2 * b0 + b1) / 3);
                colors[2][3] = 255;
                colors[3][0] = quint8((r0 + 2 * r1) / 3);
                colors[3][1] = quint8((g0 + 2 * g1) / 3);
                colors[3][2] = quint8((b0 + 2 * b1) / 3);
                colors[3][3] = 255;

                quint32 code = color[4] | (color[5] << 8) | (color[6] << 16) | (color[7] << 24);
                for (quint32 py = 0; py < 4; ++py)
                {
                    for (quint32 px = 0; px < 4; ++px)
                    {
                        const quint32 aIdx = py * 4 + px;
                        const quint8 aByte = alpha[aIdx / 2];
                        const quint8 aNib = (aIdx % 2 == 0) ? (aByte & 0x0F) : (aByte >> 4);
                        const quint8 a = quint8(aNib * 17);

                        const quint32 idx = code & 0x3;
                        code >>= 2;
                        const quint32 x = bx * 4 + px;
                        const quint32 y = by * 4 + py;
                        if (x >= w || y >= h) continue;
                        const size_t dst = (size_t(y) * size_t(w) + x) * 4;
                        out[dst + 0] = colors[idx][0];
                        out[dst + 1] = colors[idx][1];
                        out[dst + 2] = colors[idx][2];
                        out[dst + 3] = a;
                    }
                }
            }
        }
    }

    static void decodeDxt5(const quint8* src, quint32 w, quint32 h, std::vector<quint8>& out)
    {
        out.assign(size_t(w) * size_t(h) * 4, 0);
        const quint32 blocksX = (w + 3) / 4;
        const quint32 blocksY = (h + 3) / 4;

        for (quint32 by = 0; by < blocksY; ++by)
        {
            for (quint32 bx = 0; bx < blocksX; ++bx)
            {
                const quint8* block = src + (by * blocksX + bx) * 16;
                const quint8 a0 = block[0];
                const quint8 a1 = block[1];
                const quint8* aBits = block + 2;

                quint8 alpha[8];
                alpha[0] = a0;
                alpha[1] = a1;
                if (a0 > a1)
                {
                    alpha[2] = quint8((6 * a0 + 1 * a1) / 7);
                    alpha[3] = quint8((5 * a0 + 2 * a1) / 7);
                    alpha[4] = quint8((4 * a0 + 3 * a1) / 7);
                    alpha[5] = quint8((3 * a0 + 4 * a1) / 7);
                    alpha[6] = quint8((2 * a0 + 5 * a1) / 7);
                    alpha[7] = quint8((1 * a0 + 6 * a1) / 7);
                }
                else
                {
                    alpha[2] = quint8((4 * a0 + 1 * a1) / 5);
                    alpha[3] = quint8((3 * a0 + 2 * a1) / 5);
                    alpha[4] = quint8((2 * a0 + 3 * a1) / 5);
                    alpha[5] = quint8((1 * a0 + 4 * a1) / 5);
                    alpha[6] = 0;
                    alpha[7] = 255;
                }

                const quint16 c0 = quint16(block[8] | (block[9] << 8));
                const quint16 c1 = quint16(block[10] | (block[11] << 8));
                quint8 r0=0,g0=0,b0=0,r1=0,g1=0,b1=0;
                color565(c0, r0, g0, b0);
                color565(c1, r1, g1, b1);

                quint8 colors[4][4];
                colors[0][0]=r0; colors[0][1]=g0; colors[0][2]=b0; colors[0][3]=255;
                colors[1][0]=r1; colors[1][1]=g1; colors[1][2]=b1; colors[1][3]=255;
                colors[2][0] = quint8((2 * r0 + r1) / 3);
                colors[2][1] = quint8((2 * g0 + g1) / 3);
                colors[2][2] = quint8((2 * b0 + b1) / 3);
                colors[2][3] = 255;
                colors[3][0] = quint8((r0 + 2 * r1) / 3);
                colors[3][1] = quint8((g0 + 2 * g1) / 3);
                colors[3][2] = quint8((b0 + 2 * b1) / 3);
                colors[3][3] = 255;

                quint32 code = block[12] | (block[13] << 8) | (block[14] << 16) | (block[15] << 24);
                quint64 aCode = 0;
                for (int i = 0; i < 6; ++i)
                    aCode |= (quint64(aBits[i]) << (8 * i));

                for (quint32 py = 0; py < 4; ++py)
                {
                    for (quint32 px = 0; px < 4; ++px)
                    {
                        const quint32 aIdx = quint32(aCode & 0x7);
                        aCode >>= 3;
                        const quint32 idx = code & 0x3;
                        code >>= 2;
                        const quint32 x = bx * 4 + px;
                        const quint32 y = by * 4 + py;
                        if (x >= w || y >= h) continue;
                        const size_t dst = (size_t(y) * size_t(w) + x) * 4;
                        out[dst + 0] = colors[idx][0];
                        out[dst + 1] = colors[idx][1];
                        out[dst + 2] = colors[idx][2];
                        out[dst + 3] = alpha[aIdx];
                    }
                }
            }
        }
    }
}

namespace BlpLoader
{
    static bool LoadFromBytesInternal(const QByteArray& bytes, QImage* outImage, QString* outError)
    {
        if (!outImage)
        {
            setErr(outError, "Output image is null.");
            return false;
        }

        if (bytes.size() < 8)
        {
            setErr(outError, "File too small.");
            return false;
        }

        Reader r;
        r.data = reinterpret_cast<const unsigned char*>(bytes.constData());
        r.size = bytes.size();
        r.pos = 0;

        char magic[4] = {};
        if (!r.readBytes(magic, 4))
        {
            setErr(outError, "Failed reading magic.");
            return false;
        }

        const bool isBLP0 = (memcmp(magic, "BLP0", 4) == 0);
        const bool isBLP1 = (memcmp(magic, "BLP1", 4) == 0);
        const bool isBLP2 = (memcmp(magic, "BLP2", 4) == 0);
        if (!isBLP0 && !isBLP1 && !isBLP2)
        {
            setErr(outError, "Not a BLP file (missing BLP0/BLP1/BLP2 magic).");
            return false;
        }

        const int version = isBLP2 ? 2 : (isBLP1 ? 1 : 0);

        quint32 content = 0;
        if (!r.readU32(content))
        {
            setErr(outError, "Failed reading content.");
            return false;
        }

        quint32 alphaBits = 0;
        quint8 alphaType = 0;
        quint8 encodingType = 0, sampleType = 0, hasMipmaps_u8 = 0, alphaBits_u8 = 0;
        if (version >= 2)
        {
            if (!r.readU8(encodingType) || !r.readU8(alphaBits_u8) || !r.readU8(sampleType) || !r.readU8(hasMipmaps_u8))
            {
                setErr(outError, "Failed reading BLP2 fields.");
                return false;
            }
            // BLP2 layout: alphaBits, alphaType, hasMipmaps, unknown
            alphaBits = encodingType;
            alphaType = alphaBits_u8;
        }
        else
        {
            if (!r.readU32(alphaBits))
            {
                setErr(outError, "Failed reading alphaBits.");
                return false;
            }
        }

        quint32 width = 0, height = 0;
        if (!r.readU32(width) || !r.readU32(height))
        {
            setErr(outError, "Failed reading dimensions.");
            return false;
        }

        quint32 extra = 0;
        quint32 hasMipmaps = 0;
        if (version < 2)
        {
            if (!r.readU32(extra) || !r.readU32(hasMipmaps))
            {
                setErr(outError, "Failed reading BLP1 extra/hasMipmaps.");
                return false;
            }
        }
        else
        {
            hasMipmaps = hasMipmaps_u8;
        }

        // Mipmap locator (versions >= 1)
        quint32 mmOffsets[16] = {};
        quint32 mmSizes[16] = {};
        if (version >= 1)
        {
            for (int i = 0; i < 16; ++i) if (!r.readU32(mmOffsets[i])) { setErr(outError, "Failed reading mipmap offsets."); return false; }
            for (int i = 0; i < 16; ++i) if (!r.readU32(mmSizes[i]))   { setErr(outError, "Failed reading mipmap sizes."); return false; }
        }
        else
        {
            setErr(outError, "BLP0 is not supported in this previewer.");
            return false;
        }

        // Content header
        QByteArray jpegHeader;
        quint32 palette[256] = {};
        const quint32 compression = (version >= 2) ? content : 0;
        const bool isJpeg = (version >= 2) ? (compression == 0) : (content == 0);
        const bool isPaletted = (version >= 2) ? (compression == 2) : (content == 1);
        const bool isDxt = (version >= 2) ? (compression == 1) : false;

        if (isJpeg) // JPEG
        {
            quint32 jpegHeaderSize = 0;
            if (!r.readU32(jpegHeaderSize))
            {
                setErr(outError, "Failed reading JPEG header size.");
                return false;
            }
            if (!r.canRead(jpegHeaderSize))
            {
                setErr(outError, "Invalid JPEG header size.");
                return false;
            }
            jpegHeader.resize(int(jpegHeaderSize));
            if (jpegHeaderSize > 0 && !r.readBytes(jpegHeader.data(), jpegHeaderSize))
            {
                setErr(outError, "Failed reading JPEG header chunk.");
                return false;
            }
        }
        else if (isPaletted) // Paletted
        {
            for (int i = 0; i < 256; ++i)
            {
                if (!r.readU32(palette[i]))
                {
                    setErr(outError, "Failed reading palette.");
                    return false;
                }
            }
        }
        else
        {
            setErr(outError, QString("Unsupported BLP content type: %1").arg(content));
            return false;
        }

        const quint32 off0 = mmOffsets[0];
        const quint32 size0 = mmSizes[0];
        if (off0 == 0 || size0 == 0)
        {
            setErr(outError, "Missing mipmap 0 data.");
            return false;
        }
        if (off0 + size0 > (quint32)bytes.size())
        {
            setErr(outError, "Mipmap 0 range is out of file bounds.");
            return false;
        }

        const unsigned char* mip0 = reinterpret_cast<const unsigned char*>(bytes.constData()) + off0;

        if (isPaletted)
        {
            if (width == 0 || height == 0)
            {
                setErr(outError, "Invalid dimensions.");
                return false;
            }
            const quint64 pixelCount64 = quint64(width) * quint64(height);
            if (pixelCount64 > 0x7FFFFFFF)
            {
                setErr(outError, "Image too large.");
                return false;
            }
            const quint32 pixelCount = quint32(pixelCount64);
            const quint32 alphaLen = (quint32)((pixelCount64 * quint64(alphaBits) + 7) / 8);
            const quint64 needed = quint64(pixelCount) + quint64(alphaLen);
            if (needed > size0)
            {
                setErr(outError, "Direct mipmap data too small for expected pixel count.");
                return false;
            }

            const quint8* idxData = reinterpret_cast<const quint8*>(mip0);
            const quint8* alphaData = (alphaLen > 0) ? (idxData + pixelCount) : nullptr;

            QImage img(int(width), int(height), QImage::Format_RGBA8888);
            if (img.isNull())
            {
                setErr(outError, "Failed creating QImage.");
                return false;
            }

            // Fill pixels row-major.
            quint8* dst = img.bits();
            const int stride = img.bytesPerLine();
            for (quint32 y = 0; y < height; ++y)
            {
                quint8* row = dst + int(y) * stride;
                for (quint32 x = 0; x < width; ++x)
                {
                    const quint32 i = y * width + x;
                    const quint8 palIndex = idxData[i];
                    const quint32 p = palette[palIndex];
                    // BLP palette entries are stored as BGRA.
                    const quint8 bC = quint8(p & 0xFF);
                    const quint8 gC = quint8((p >> 8) & 0xFF);
                    const quint8 rC = quint8((p >> 16) & 0xFF);
                    const quint8 aC = alphaForPixel(alphaData, alphaBits, i);

                    // RGBA8888
                    row[x * 4 + 0] = rC;
                    row[x * 4 + 1] = gC;
                    row[x * 4 + 2] = bC;
                    row[x * 4 + 3] = aC;
                }
            }

            *outImage = img;
            return true;
        }
        else if (isDxt)
        {
            if (width == 0 || height == 0)
            {
                setErr(outError, "Invalid dimensions.");
                return false;
            }
            std::vector<quint8> rgba;
            if (alphaType == 0)
                decodeDxt1(mip0, width, height, rgba);
            else if (alphaType == 1)
                decodeDxt3(mip0, width, height, rgba);
            else if (alphaType == 7)
                decodeDxt5(mip0, width, height, rgba);
            else
                decodeDxt1(mip0, width, height, rgba);

            QImage img(int(width), int(height), QImage::Format_RGBA8888);
            if (img.isNull())
            {
                setErr(outError, "Failed creating QImage for DXT.");
                return false;
            }
            const int stride = img.bytesPerLine();
            const quint8* src = rgba.data();
            for (quint32 y = 0; y < height; ++y)
            {
                memcpy(img.bits() + y * stride,
                       src + size_t(y) * size_t(width) * 4,
                       size_t(width) * 4);
            }
            *outImage = img;
            return true;
        }
        else
        {
            // JPEG content: concatenate header + chunk and let Qt decode.
            QByteArray jpegData;
            jpegData.reserve(jpegHeader.size() + int(size0));
            jpegData.append(jpegHeader);
            jpegData.append(reinterpret_cast<const char*>(mip0), int(size0));

            QImage img;
            if (!img.loadFromData(jpegData, "JPG") && !img.loadFromData(jpegData, "JPEG"))
            {
                setErr(outError, "Qt failed to decode JPEG content BLP (this may be non-standard BGRA JPEG).");
                return false;
            }

            if (img.format() != QImage::Format_RGBA8888)
                img = img.convertToFormat(QImage::Format_RGBA8888);

            // BLP JPEG payloads are typically BGR; match RMS/War3BatchPreview swap.
            img = img.rgbSwapped();

            *outImage = img;
            return true;
        }
    }

    bool LoadBlpToImage(const QString& filePath, QImage* outImage, QString* outError)
    {
        QFile f(filePath);
        if (!f.open(QIODevice::ReadOnly))
        {
            setErr(outError, QString("Failed to open: %1").arg(filePath));
            return false;
        }
        const QByteArray bytes = f.readAll();
        return LoadFromBytesInternal(bytes, outImage, outError);
    }

    bool LoadBlpToImageCached(const QString& filePath, QImage* outImage, QString* outError)
    {
        if (!outImage)
        {
            setErr(outError, "Output image is null.");
            return false;
        }

        static QHash<QString, QImage> cache;
        static QMutex cacheMutex;

        {
            QMutexLocker lock(&cacheMutex);
            auto it = cache.constFind(filePath);
            if (it != cache.constEnd())
            {
                *outImage = it.value();
                return !outImage->isNull();
            }
        }

        QImage img;
        QString err;
        if (!LoadBlpToImage(filePath, &img, &err))
        {
            setErr(outError, err);
            return false;
        }

        {
            QMutexLocker lock(&cacheMutex);
            cache.insert(filePath, img);
        }
        *outImage = img;
        return true;
    }

    bool LoadBlpToImageFromBytes(const QByteArray& bytes, QImage* outImage, QString* outError)
    {
        return LoadFromBytesInternal(bytes, outImage, outError);
    }
}
