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
}

namespace BlpLoader
{
    bool LoadBlpToImage(const QString& filePath, QImage* outImage, QString* outError)
    {
        if (!outImage)
        {
            setErr(outError, "Output image is null.");
            return false;
        }

        QFile f(filePath);
        if (!f.open(QIODevice::ReadOnly))
        {
            setErr(outError, QString("Failed to open: %1").arg(filePath));
            return false;
        }

        const QByteArray bytes = f.readAll();
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
        quint8 encodingType = 0, sampleType = 0, hasMipmaps_u8 = 0, alphaBits_u8 = 0;
        if (version >= 2)
        {
            if (!r.readU8(encodingType) || !r.readU8(alphaBits_u8) || !r.readU8(sampleType) || !r.readU8(hasMipmaps_u8))
            {
                setErr(outError, "Failed reading BLP2 fields.");
                return false;
            }
            alphaBits = alphaBits_u8;
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
        if (content == 0) // JPEG
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
        else if (content == 1) // Direct
        {
            if (version >= 2 && encodingType != 1)
            {
                // BLP2 can store DXT etc; we only support palettized here.
                setErr(outError, "BLP2 direct encoding is not supported (expected palettized encodingType=1)."
                                 " Consider converting textures to BLP1 palettized or .png for preview.");
                return false;
            }
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

        if (content == 1)
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
                    const quint8 rC = quint8(p & 0xFF);
                    const quint8 gC = quint8((p >> 8) & 0xFF);
                    const quint8 bC = quint8((p >> 16) & 0xFF);
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

            *outImage = img;
            return true;
        }
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
}
