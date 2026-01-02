#include "MdxLoader.h"

#include <QFile>
#include <QStringList>
#include <QtGlobal>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "LogSink.h"
namespace
{
    struct Reader
    {
        const unsigned char* data = nullptr;
        qsizetype size = 0;
        qsizetype pos = 0;

        bool canRead(qsizetype n) const { return pos + n <= size; }

        bool readBytes(void* out, qsizetype n)
        {
            if (!canRead(n)) return false;
            memcpy(out, data + pos, size_t(n));
            pos += n;
            return true;
        }

        bool readU16(quint16& v)
        {
            if (!canRead(2)) return false;
            v = (quint16)data[pos] | ((quint16)data[pos + 1] << 8);
            pos += 2;
            return true;
        }

        bool readU32(quint32& v)
        {
            if (!canRead(4)) return false;
            v = (quint32)data[pos] |
                ((quint32)data[pos + 1] << 8) |
                ((quint32)data[pos + 2] << 16) |
                ((quint32)data[pos + 3] << 24);
            pos += 4;
            return true;
        }

        bool readI32(qint32& v)
        {
            quint32 u = 0;
            if (!readU32(u)) return false;
            v = qint32(u);
            return true;
        }

        bool readF32(float& v)
        {
            quint32 u = 0;
            if (!readU32(u)) return false;
            static_assert(sizeof(float) == 4, "float must be 4 bytes");
            memcpy(&v, &u, 4);
            return true;
        }

        bool readTag(char out[4])
        {
            return readBytes(out, 4);
        }

        bool peekTag(char out[4]) const
        {
            if (pos + 4 > size) return false;
            memcpy(out, data + pos, 4);
            return true;
        }

        bool skip(qsizetype n)
        {
            if (!canRead(n)) return false;
            pos += n;
            return true;
        }
    };

    static void setErr(QString* outError, const QString& msg)
    {
        if (outError) *outError = msg;
    }

    static bool tagEq(const char t[4], const char* s)
    {
        return t[0] == s[0] && t[1] == s[1] && t[2] == s[2] && t[3] == s[3];
    }

    static std::string readFixedString(Reader& r, qsizetype n)
    {
        std::string out;
        out.resize(size_t(n));
        if (!r.readBytes(out.data(), n)) return {};
        // Trim at first NUL
        const auto nul = out.find('\0');
        if (nul != std::string::npos) out.resize(nul);
        // Trim trailing spaces
        while (!out.empty() && (out.back() == '\0' || out.back() == ' ')) out.pop_back();
        return out;
    }

    static bool readArrayTagCount(Reader& r, const char* expected, quint32& outCount, QString* outError)
    {
        char t[4] = {};
        if (!r.readTag(t))
        {
            setErr(outError, QString("Unexpected EOF reading tag for %1").arg(expected));
            return false;
        }
        if (!tagEq(t, expected))
        {
            setErr(outError, QString("Expected tag '%1' but got '%2%3%4%5'")
                            .arg(expected)
                            .arg(QChar(t[0])).arg(QChar(t[1])).arg(QChar(t[2])).arg(QChar(t[3])));
            return false;
        }
        if (!r.readU32(outCount))
        {
            setErr(outError, QString("Unexpected EOF reading count for %1").arg(expected));
            return false;
        }
        return true;
    }

    static quint32 indexCountToPrimitiveCount(quint32 d3dPrimitiveType, quint32 indexCount)
    {
        // MDX PCNT stores index counts per group (see MdxLib).
        switch (d3dPrimitiveType)
        {
        case 4: // triangle list
            return indexCount / 3;
        case 5: // triangle strip
        case 6: // triangle fan
            return (indexCount >= 2) ? (indexCount - 2) : 0;
        case 7: // quad list
            return indexCount / 4;
        default:
            return 0;
        }
    }

    static void appendTrianglesFromPrimitive(
        quint32 type,
        quint32 primCount,
        const std::vector<quint16>& srcIndices,
        quint32 start,
        quint32 baseVertex,
        std::vector<std::uint32_t>& outTriIndices)
    {
        if (primCount == 0) return;

        if (type == 4)
        {
            // triangle list
            for (quint32 t = 0; t < primCount; ++t)
            {
                const quint32 i0 = start + t * 3 + 0;
                const quint32 i1 = start + t * 3 + 1;
                const quint32 i2 = start + t * 3 + 2;
                if (i2 >= srcIndices.size()) break;
                outTriIndices.push_back(baseVertex + srcIndices[i0]);
                outTriIndices.push_back(baseVertex + srcIndices[i1]);
                outTriIndices.push_back(baseVertex + srcIndices[i2]);
            }
        }
        else if (type == 5)
        {
            // triangle strip: D3D-style winding alternates each triangle
            for (quint32 t = 0; t < primCount; ++t)
            {
                const quint32 i0 = start + t;
                const quint32 i1 = start + t + 1;
                const quint32 i2 = start + t + 2;
                if (i2 >= srcIndices.size()) break;
                const std::uint32_t a = baseVertex + srcIndices[i0];
                const std::uint32_t b = baseVertex + srcIndices[i1];
                const std::uint32_t c = baseVertex + srcIndices[i2];
                if ((t & 1u) == 0)
                {
                    outTriIndices.push_back(a);
                    outTriIndices.push_back(b);
                    outTriIndices.push_back(c);
                }
                else
                {
                    outTriIndices.push_back(b);
                    outTriIndices.push_back(a);
                    outTriIndices.push_back(c);
                }
            }
        }
        else if (type == 6)
        {
            // triangle fan
            if (start + 2 >= srcIndices.size()) return;
            const std::uint32_t center = baseVertex + srcIndices[start];
            for (quint32 t = 0; t < primCount; ++t)
            {
                const quint32 i1 = start + t + 1;
                const quint32 i2 = start + t + 2;
                if (i2 >= srcIndices.size()) break;
                outTriIndices.push_back(center);
                outTriIndices.push_back(baseVertex + srcIndices[i1]);
                outTriIndices.push_back(baseVertex + srcIndices[i2]);
            }
        }
        else if (type == 7)
        {
            // quads -> 2 triangles
            for (quint32 q = 0; q < primCount; ++q)
            {
                const quint32 i0 = start + q * 4 + 0;
                const quint32 i1 = start + q * 4 + 1;
                const quint32 i2 = start + q * 4 + 2;
                const quint32 i3 = start + q * 4 + 3;
                if (i3 >= srcIndices.size()) break;
                const std::uint32_t a = baseVertex + srcIndices[i0];
                const std::uint32_t b = baseVertex + srcIndices[i1];
                const std::uint32_t c = baseVertex + srcIndices[i2];
                const std::uint32_t d = baseVertex + srcIndices[i3];
                outTriIndices.push_back(a); outTriIndices.push_back(b); outTriIndices.push_back(c);
                outTriIndices.push_back(a); outTriIndices.push_back(c); outTriIndices.push_back(d);
            }
        }
        // other primitive types ignored for preview
    }

    struct GeosetParsed
    {
        std::vector<ModelVertex> vertices;
        std::vector<std::uint32_t> triIndices;
        quint32 materialId = 0;
        std::vector<std::uint8_t> vertexGroups;
        std::vector<ModelData::SkinGroup> groups;
        std::vector<std::uint8_t> gndxRaw;
        std::vector<std::uint32_t> mtgcRaw;
        std::vector<std::int32_t> matsRaw;
        std::vector<std::vector<std::int32_t>> expandedGroups;
    };

    static bool parseGeoset(Reader& r, quint32 inclusiveSize, quint32 mdxVersion, GeosetParsed& out, QString* outError)
    {
        // We have already consumed the inclusiveSize field from the parent reader.
        const qsizetype geosetDataSize = qsizetype(inclusiveSize) - 4;
        if (geosetDataSize < 0 || !r.canRead(geosetDataSize))
        {
            setErr(outError, "Geoset size out of bounds.");
            return false;
        }

        Reader gs;
        gs.data = r.data + r.pos;
        gs.size = geosetDataSize;
        gs.pos = 0;

        // Advance parent reader over this geoset.
        r.pos += geosetDataSize;

        // VRTX
        quint32 vertexCount = 0;
        if (!readArrayTagCount(gs, "VRTX", vertexCount, outError)) return false;
        out.vertices.resize(vertexCount);
        for (quint32 i = 0; i < vertexCount; ++i)
        {
            float x = 0, y = 0, z = 0;
            if (!gs.readF32(x) || !gs.readF32(y) || !gs.readF32(z))
            {
                setErr(outError, "Unexpected EOF reading VRTX vertices.");
                return false;
            }
            out.vertices[i].px = x;
            out.vertices[i].py = y;
            out.vertices[i].pz = z;
        }

        // NRMS
        quint32 normalCount = 0;
        if (!readArrayTagCount(gs, "NRMS", normalCount, outError)) return false;
        if (normalCount != vertexCount)
        {
            // still try to read min(vertexCount, normalCount)
        }
        const quint32 nRead = std::min(vertexCount, normalCount);
        for (quint32 i = 0; i < nRead; ++i)
        {
            float x = 0, y = 0, z = 1;
            if (!gs.readF32(x) || !gs.readF32(y) || !gs.readF32(z))
            {
                setErr(outError, "Unexpected EOF reading NRMS normals.");
                return false;
            }
            out.vertices[i].nx = x;
            out.vertices[i].ny = y;
            out.vertices[i].nz = z;
        }
        // Skip any extra normals if present
        for (quint32 i = nRead; i < normalCount; ++i)
        {
            float tmp = 0;
            if (!gs.readF32(tmp) || !gs.readF32(tmp) || !gs.readF32(tmp))
                break;
        }

        // PTYP
        quint32 faceTypeGroupsCount = 0;
        if (!readArrayTagCount(gs, "PTYP", faceTypeGroupsCount, outError)) return false;
        std::vector<quint32> faceTypeGroups(faceTypeGroupsCount);
        for (quint32 i = 0; i < faceTypeGroupsCount; ++i)
        {
            if (!gs.readU32(faceTypeGroups[i]))
            {
                setErr(outError, "Unexpected EOF reading PTYP.");
                return false;
            }
        }

        // PCNT
        quint32 faceGroupsCount = 0;
        if (!readArrayTagCount(gs, "PCNT", faceGroupsCount, outError)) return false;
        std::vector<quint32> faceGroups(faceGroupsCount);
        for (quint32 i = 0; i < faceGroupsCount; ++i)
        {
            if (!gs.readU32(faceGroups[i]))
            {
                setErr(outError, "Unexpected EOF reading PCNT.");
                return false;
            }
        }

        // PVTX
        quint32 facesCount = 0;
        if (!readArrayTagCount(gs, "PVTX", facesCount, outError)) return false;
        std::vector<quint16> faces(facesCount);
        for (quint32 i = 0; i < facesCount; ++i)
        {
            quint16 v = 0;
            if (!gs.readU16(v))
            {
                setErr(outError, "Unexpected EOF reading PVTX.");
                return false;
            }
            faces[i] = v;
        }

        // GNDX (skip)
        quint32 vertexGroupCount = 0;
        if (!readArrayTagCount(gs, "GNDX", vertexGroupCount, outError)) return false;
        out.vertexGroups.resize(vertexGroupCount);
        for (quint32 i = 0; i < vertexGroupCount; ++i)
        {
            unsigned char v = 0;
            if (!gs.readBytes(&v, 1))
            {
                setErr(outError, "Unexpected EOF reading GNDX.");
                return false;
            }
            out.vertexGroups[i] = std::uint8_t(v);
        }
        out.gndxRaw = out.vertexGroups;

        // MTGC (matrix group sizes)
        quint32 matrixGroupCount = 0;
        if (!readArrayTagCount(gs, "MTGC", matrixGroupCount, outError)) return false;
        std::vector<quint32> groupSizes(matrixGroupCount);
        out.groups.reserve(matrixGroupCount);
        for (quint32 i = 0; i < matrixGroupCount; ++i)
        {
            quint32 sz = 0;
            if (!gs.readU32(sz))
            {
                setErr(outError, "Unexpected EOF reading MTGC.");
                return false;
            }
            groupSizes[i] = sz;
            out.groups.push_back(ModelData::SkinGroup{});
        }
        out.mtgcRaw.assign(groupSizes.begin(), groupSizes.end());
        if (!groupSizes.empty())
        {
            for (std::uint8_t vg : out.vertexGroups)
                Q_ASSERT(vg < groupSizes.size());
        }

        // MATS (matrix indices)
        quint32 matrixIndexCount = 0;
        if (!readArrayTagCount(gs, "MATS", matrixIndexCount, outError)) return false;
        quint32 groupIndex = 0;
        quint32 remaining = (matrixGroupCount > 0) ? groupSizes[0] : 0;
        for (quint32 i = 0; i < matrixIndexCount; ++i)
        {
            qint32 nodeIndex = -1;
            if (!gs.readI32(nodeIndex))
            {
                setErr(outError, "Unexpected EOF reading MATS.");
                return false;
            }
            out.matsRaw.push_back(nodeIndex);
            while (groupIndex < matrixGroupCount && remaining == 0)
            {
                groupIndex++;
                remaining = (groupIndex < matrixGroupCount) ? groupSizes[groupIndex] : 0;
            }
            if (groupIndex < out.groups.size())
            {
                out.groups[groupIndex].nodeIndices.push_back(nodeIndex);
                if (remaining > 0)
                    --remaining;
            }
        }

        // Expanded groups from MTGC/MATS
        out.expandedGroups.clear();
        out.expandedGroups.reserve(groupSizes.size());
        std::size_t matsOffset = 0;
        for (quint32 sz : groupSizes)
        {
            std::vector<std::int32_t> group;
            group.reserve(sz);
            for (quint32 k = 0; k < sz && matsOffset < out.matsRaw.size(); ++k, ++matsOffset)
                group.push_back(out.matsRaw[matsOffset]);
            out.expandedGroups.push_back(std::move(group));
        }
        Q_ASSERT(matsOffset <= out.matsRaw.size());

        // Fixed fields
        quint32 materialId = 0, selectionFlags = 0, selectionGroup = 0;
        if (!gs.readU32(materialId) || !gs.readU32(selectionFlags) || !gs.readU32(selectionGroup))
        {
            setErr(outError, "Unexpected EOF reading geoset header fields.");
            return false;
        }
        out.materialId = materialId;

        // Extent (min/max/radius) + extents
        // Extent is 7 floats = 28 bytes
        auto skipExtent = [&gs]() -> bool { return gs.skip(28); };

        if (!skipExtent())
        {
            setErr(outError, "Unexpected EOF skipping extent.");
            return false;
        }
        quint32 extentsCount = 0;
        if (!gs.readU32(extentsCount))
        {
            setErr(outError, "Unexpected EOF reading extents count.");
            return false;
        }
        if (!gs.skip(qsizetype(extentsCount) * 28))
        {
            setErr(outError, "Unexpected EOF skipping extents.");
            return false;
        }

        if (mdxVersion > 800)
        {
            // LOD fields present for Reforged-era versions; skip for completeness.
            if (!gs.skip(4 + 80))
            {
                setErr(outError, "Unexpected EOF skipping LOD fields.");
                return false;
            }
            // optional tangents/skin chunks could appear here
            char peek[4] = {};
            while (gs.peekTag(peek) && (tagEq(peek, "TANG") || tagEq(peek, "SKIN")))
            {
                char tag[4] = {};
                quint32 sz = 0;
                gs.readTag(tag);
                if (!gs.readU32(sz) || !gs.skip(qsizetype(sz)))
                    break;
            }
        }

        // UVAS
        quint32 uvSetCount = 0;
        if (!readArrayTagCount(gs, "UVAS", uvSetCount, outError)) return false;
        // Each set begins with UVBS
        for (quint32 s = 0; s < uvSetCount; ++s)
        {
            quint32 tcCount = 0;
            if (!readArrayTagCount(gs, "UVBS", tcCount, outError)) return false;
            const quint32 pairsToRead = std::min(tcCount, vertexCount);
            for (quint32 i = 0; i < pairsToRead; ++i)
            {
                float u = 0, v = 0;
                if (!gs.readF32(u) || !gs.readF32(v))
                {
                    setErr(outError, "Unexpected EOF reading UVBS.");
                    return false;
                }
                if (s == 0 && i < out.vertices.size())
                {
                    out.vertices[i].u = u;
                    out.vertices[i].v = 1.0f - v;
                }
            }
            // Skip extra coords if tcCount > vertexCount
            if (tcCount > pairsToRead)
            {
                const qsizetype extraPairs = qsizetype(tcCount - pairsToRead);
                if (!gs.skip(extraPairs * 8))
                {
                    setErr(outError, "Unexpected EOF skipping extra UVs.");
                    return false;
                }
            }
        }

        // Convert primitives to triangle list
        const quint32 groups = std::min(faceTypeGroupsCount, faceGroupsCount);
        quint32 cursor = 0;
        for (quint32 g = 0; g < groups; ++g)
        {
            const quint32 type = faceTypeGroups[g];
            const quint32 idxCount = faceGroups[g];
            const quint32 primCount = indexCountToPrimitiveCount(type, idxCount);
            if (idxCount == 0) { continue; }
            if (cursor + idxCount > faces.size()) break;
            if (primCount > 0)
                appendTrianglesFromPrimitive(type, primCount, faces, cursor, /*baseVertex*/ 0, out.triIndices);
            cursor += idxCount;
        }

        return true;
    }

    static bool parseTextures(Reader& r, quint32 chunkSize, ModelData& out, QString* outError)
    {
        // Texture record is 4 + 260 + 4 bytes = 268
        const qsizetype recordSize = 4 + 260 + 4;
        if (chunkSize % recordSize != 0)
        {
            // still try best-effort
        }
        const qsizetype count = qsizetype(chunkSize) / recordSize;
        out.textures.reserve(size_t(count));
        for (qsizetype i = 0; i < count; ++i)
        {
            ModelTexture t;
            quint32 rep = 0, flags = 0;
            if (!r.readU32(rep)) { setErr(outError, "Unexpected EOF in TEXS."); return false; }
            const std::string name = readFixedString(r, 260);
            if (!r.readU32(flags)) { setErr(outError, "Unexpected EOF in TEXS."); return false; }
            t.replaceableId = rep;
            t.fileName = name;
            t.flags = flags;
            out.textures.push_back(std::move(t));
        }
        return true;
    }

    static bool parseMaterials(Reader& r, quint32 chunkSize, quint32 mdxVersion, ModelData& out, QString* outError)
    {
        const qsizetype end = r.pos + qsizetype(chunkSize);
        while (r.pos + 4 <= end)
        {
            const qsizetype matStart = r.pos;
            quint32 inclusiveSize = 0;
            if (!r.readU32(inclusiveSize)) break;
            if (inclusiveSize < 12 || matStart + qsizetype(inclusiveSize) > end)
            {
                setErr(outError, "Invalid material inclusiveSize.");
                return false;
            }

            Reader mr;
            mr.data = r.data + matStart + 4;
            mr.size = qsizetype(inclusiveSize) - 4;
            mr.pos = 0;

            // advance outer reader
            r.pos = matStart + qsizetype(inclusiveSize);

            ModelMaterial m;
            qint32 priorityPlane = 0;
            quint32 flags = 0;
            if (!mr.readI32(priorityPlane) || !mr.readU32(flags))
            {
                setErr(outError, "Unexpected EOF in material header.");
                return false;
            }
            m.priorityPlane = int(priorityPlane);
            m.flags = flags;

            if (mdxVersion > 800)
            {
                // shader[80]
                if (!mr.skip(80))
                {
                    setErr(outError, "Unexpected EOF skipping material shader.");
                    return false;
                }
            }

            // LAYS
            char lays[4] = {};
            if (!mr.readTag(lays) || !tagEq(lays, "LAYS"))
            {
                // Some files might be malformed; just store default layer.
                out.materials.push_back(m);
                continue;
            }
            quint32 layersCount = 0;
            if (!mr.readU32(layersCount))
            {
                setErr(outError, "Unexpected EOF reading LAYS count.");
                return false;
            }

            if (layersCount == 0)
            {
                out.materials.push_back(m);
                continue;
            }

            // Parse first layer, skip the rest
            for (quint32 li = 0; li < layersCount; ++li)
            {
                const qsizetype layerStart = mr.pos;
                quint32 layerSize = 0;
                if (!mr.readU32(layerSize)) { setErr(outError, "Unexpected EOF reading layer size."); return false; }
                if (layerSize < 24 || layerStart + qsizetype(layerSize) > mr.size)
                {
                    setErr(outError, "Invalid layer size.");
                    return false;
                }

                // Base fields
                ModelLayer layer;
                quint32 shadingFlags = 0, filterMode = 0;
                if (!mr.readU32(shadingFlags) || !mr.readU32(filterMode))
                {
                    setErr(outError, "Unexpected EOF reading layer header.");
                    return false;
                }
                layer.shadingFlags = shadingFlags;
                layer.filterMode = filterMode;

                const qsizetype layerEnd = layerStart + qsizetype(layerSize);
                if (mdxVersion > 800 && (layerEnd - mr.pos) >= 20)
                {
                    // Optional layerShaderId in reforged-era files.
                    quint32 layerShaderId = 0;
                    if (!mr.readU32(layerShaderId))
                    {
                        setErr(outError, "Unexpected EOF reading layerShaderId.");
                        return false;
                    }
                }

                quint32 textureId = 0, textureAnimId = 0, coordId = 0;
                float alpha = 1.0f;
                if (!mr.readU32(textureId) || !mr.readU32(textureAnimId) || !mr.readU32(coordId) ||
                    !mr.readF32(alpha))
                {
                    setErr(outError, "Unexpected EOF reading layer fields.");
                    return false;
                }
                layer.textureId = textureId;
                layer.coordId = coordId;
                layer.alpha = alpha;

                // Optional emissive gain if present, then skip remaining tracks.
                if ((layerEnd - mr.pos) >= 4)
                {
                    float emissiveGain = 0.0f;
                    mr.readF32(emissiveGain);
                }

                // Skip remaining unknown tracks to end of layer
                mr.pos = layerEnd;

                if (li == 0)
                    m.layer = layer;
            }

            out.materials.push_back(m);
        }

        return true;
    }

    static bool parseSequences(Reader& r, quint32 chunkSize, ModelData& out, QString* outError)
    {
        Q_UNUSED(outError);
        // SEQS is an array of fixed-size records.
        // Each sequence is 132 bytes in MDX 800 (most common):
        // char name[80]; int32 start; int32 end; float moveSpeed; uint32 flags;
        // int32 rarity; int32 syncPoint; float extents[6] (min/max) => 24 bytes.
        // Total: 80 + 4 + 4 + 4 + 4 + 4 + 4 + 24 = 128? Some docs list 132; in practice extra 4 bytes padding.
        // We parse conservatively based on remaining bytes.

        const qsizetype recordMin = 80 + 4 + 4 + 4 + 4; // name + start/end + moveSpeed + flags
        if (chunkSize < recordMin)
            return true;

        while (r.canRead(recordMin))
        {
            ModelData::Sequence s;
            s.name = readFixedString(r, 80);
            qint32 st = 0, en = 0;
            float move = 0.0f;
            quint32 flags = 0;
            if (!r.readI32(st) || !r.readI32(en) || !r.readF32(move) || !r.readU32(flags))
                break;
            s.startMs = (st < 0) ? 0u : std::uint32_t(st);
            s.endMs = (en < 0) ? 0u : std::uint32_t(en);
            s.moveSpeed = move;
            s.flags = flags;

            // Skip the rest of the record if present.
            // Try to skip rarity + syncPoint + extents + optional padding.
            // Many files use 132-byte records.
            const qsizetype consumed = recordMin;
            // We already consumed recordMin; now attempt to skip up to 132-recordMin, but only if available.
            const qsizetype targetRecordSize = 132;
            const qsizetype remainForThis = std::min<qsizetype>(targetRecordSize - consumed, r.size - r.pos);
            if (remainForThis > 0 && r.canRead(remainForThis))
                r.skip(remainForThis);

            out.sequences.push_back(std::move(s));
        }
        return true;
    }

    static bool parseGlobalSequences(Reader& r, quint32 chunkSize, ModelData& out, QString* outError)
    {
        Q_UNUSED(outError);
        const qsizetype n = chunkSize / 4;
        out.globalSequencesMs.reserve(out.globalSequencesMs.size() + size_t(n));
        for (qsizetype i = 0; i < n; ++i)
        {
            quint32 v = 0;
            if (!r.readU32(v)) break;
            out.globalSequencesMs.push_back(v);
        }
        return true;
    }

    static bool parsePivots(Reader& r, quint32 chunkSize, ModelData& out, QString* outError)
    {
        Q_UNUSED(outError);
        const qsizetype n = chunkSize / (qsizetype(3) * 4);
        out.pivots.reserve(out.pivots.size() + size_t(n));
        for (qsizetype i = 0; i < n; ++i)
        {
            ModelData::Pivot p;
            if (!r.readF32(p.x) || !r.readF32(p.y) || !r.readF32(p.z)) break;
            out.pivots.push_back(p);
        }
        return true;
    }

    static bool parseFloatTrack(Reader& r, quint32 chunkSize, ModelData::MdxTrack<float>& outTrack)
    {
        // Header: int32 numTracks, int32 interpolationType, int32 globalSeqId
        qint32 num = 0, interp = 0, globalSeq = -1;
        if (!r.readI32(num) || !r.readI32(interp) || !r.readI32(globalSeq))
            return false;
        if (num < 0) num = 0;

        outTrack.globalSeqId = globalSeq;
        outTrack.keys.clear();
        outTrack.keys.reserve(size_t(num));

        // Map interpolation type
        switch (interp)
        {
        case 0: outTrack.interp = ModelData::MdxInterp::None; break;
        case 1: outTrack.interp = ModelData::MdxInterp::Linear; break;
        case 2: outTrack.interp = ModelData::MdxInterp::Hermite; break;
        case 3: outTrack.interp = ModelData::MdxInterp::Bezier; break;
        default: outTrack.interp = ModelData::MdxInterp::None; break;
        }

        for (qint32 i = 0; i < num; ++i)
        {
            qint32 t = 0;
            float v = 0.0f;
            if (!r.readI32(t) || !r.readF32(v))
                return false;
            ModelData::MdxTrackKey<float> k;
            k.timeMs = (t < 0) ? 0u : std::uint32_t(t);
            k.value = v;
            if (interp >= 2)
            {
                float inT = 0.0f, outT = 0.0f;
                if (!r.readF32(inT) || !r.readF32(outT))
                    return false;
                k.inTan = inT;
                k.outTan = outT;
            }
            outTrack.keys.push_back(k);
        }

        Q_UNUSED(chunkSize);
        return true;
    }

    static bool readVec3(Reader& r, Vec3& out)
    {
        return r.readF32(out.x) && r.readF32(out.y) && r.readF32(out.z);
    }

    static bool readVec4(Reader& r, Vec4& out)
    {
        return r.readF32(out.x) && r.readF32(out.y) && r.readF32(out.z) && r.readF32(out.w);
    }

    static bool parseVec3Track(Reader& r, ModelData::MdxTrack<Vec3>& outTrack)
    {
        qint32 num = 0, interp = 0, globalSeq = -1;
        if (!r.readI32(num) || !r.readI32(interp) || !r.readI32(globalSeq))
            return false;
        if (num < 0) num = 0;

        outTrack.globalSeqId = globalSeq;
        outTrack.keys.clear();
        outTrack.keys.reserve(size_t(num));

        switch (interp)
        {
        case 0: outTrack.interp = ModelData::MdxInterp::None; break;
        case 1: outTrack.interp = ModelData::MdxInterp::Linear; break;
        case 2: outTrack.interp = ModelData::MdxInterp::Hermite; break;
        case 3: outTrack.interp = ModelData::MdxInterp::Bezier; break;
        default: outTrack.interp = ModelData::MdxInterp::None; break;
        }

        for (qint32 i = 0; i < num; ++i)
        {
            qint32 t = 0;
            Vec3 v{};
            if (!r.readI32(t) || !readVec3(r, v))
                return false;
            ModelData::MdxTrackKey<Vec3> k;
            k.timeMs = (t < 0) ? 0u : std::uint32_t(t);
            k.value = v;
            if (interp >= 2)
            {
                Vec3 inT{}, outT{};
                if (!readVec3(r, inT) || !readVec3(r, outT))
                    return false;
                k.inTan = inT;
                k.outTan = outT;
            }
            outTrack.keys.push_back(k);
        }
        return true;
    }

    static bool parseVec4Track(Reader& r, ModelData::MdxTrack<Vec4>& outTrack)
    {
        qint32 num = 0, interp = 0, globalSeq = -1;
        if (!r.readI32(num) || !r.readI32(interp) || !r.readI32(globalSeq))
            return false;
        if (num < 0) num = 0;

        outTrack.globalSeqId = globalSeq;
        outTrack.keys.clear();
        outTrack.keys.reserve(size_t(num));

        switch (interp)
        {
        case 0: outTrack.interp = ModelData::MdxInterp::None; break;
        case 1: outTrack.interp = ModelData::MdxInterp::Linear; break;
        case 2: outTrack.interp = ModelData::MdxInterp::Hermite; break;
        case 3: outTrack.interp = ModelData::MdxInterp::Bezier; break;
        default: outTrack.interp = ModelData::MdxInterp::None; break;
        }

        for (qint32 i = 0; i < num; ++i)
        {
            qint32 t = 0;
            Vec4 v{};
            if (!r.readI32(t) || !readVec4(r, v))
                return false;
            ModelData::MdxTrackKey<Vec4> k;
            k.timeMs = (t < 0) ? 0u : std::uint32_t(t);
            k.value = v;
            if (interp >= 2)
            {
                Vec4 inT{}, outT{};
                if (!readVec4(r, inT) || !readVec4(r, outT))
                    return false;
                k.inTan = inT;
                k.outTan = outT;
            }
            outTrack.keys.push_back(k);
        }
        return true;
    }

    static bool parseNodeBlock(Reader& r, ModelData::Node& outNode, QString* outError)
    {
        const qsizetype start = r.pos;
        quint32 size = 0;
        if (!r.readU32(size))
        {
            setErr(outError, "Unexpected EOF reading node size.");
            return false;
        }
        if (size < 96 || !r.canRead(qsizetype(size) - 4))
        {
            setErr(outError, "Invalid node size.");
            return false;
        }
        const qsizetype end = start + qsizetype(size);

        outNode.name = readFixedString(r, 80);
        qint32 nodeId = -1;
        qint32 parentId = -1;
        quint32 flags = 0;
        if (!r.readI32(nodeId) || !r.readI32(parentId) || !r.readU32(flags))
        {
            setErr(outError, "Unexpected EOF reading node header.");
            return false;
        }
        outNode.nodeId = nodeId;
        outNode.parentId = parentId;
        outNode.flags = flags;

        while (r.pos + 4 <= end)
        {
            char tag[4] = {};
            if (!r.readTag(tag))
                break;
            if (tagEq(tag, "KGTR"))
            {
                if (!parseVec3Track(r, outNode.trackTranslation))
                    break;
            }
            else if (tagEq(tag, "KGRT"))
            {
                if (!parseVec4Track(r, outNode.trackRotation))
                    break;
            }
            else if (tagEq(tag, "KGSC"))
            {
                if (!parseVec3Track(r, outNode.trackScaling))
                    break;
            }
            else
            {
                LogSink::instance().log(QString("Unknown node track tag: %1%2%3%4")
                                            .arg(QChar(tag[0])).arg(QChar(tag[1]))
                                            .arg(QChar(tag[2])).arg(QChar(tag[3])));
                break;
            }
        }

        r.pos = end;
        return true;
    }

    static void storeNode(ModelData& model, ModelData::Node&& node)
    {
        if (node.nodeId < 0)
            return;
        const std::size_t idx = std::size_t(node.nodeId);
        if (model.nodes.size() <= idx)
            model.nodes.resize(idx + 1);
        model.nodes[idx] = std::move(node);
    }

    static bool parseBones(Reader& r, quint32 chunkSize, ModelData& model, QString* outError)
    {
        const qsizetype startPos = r.pos;
        const qsizetype endPos = startPos + qsizetype(chunkSize);
        while (r.pos + 4 <= endPos)
        {
            ModelData::Node node;
            if (!parseNodeBlock(r, node, outError))
            {
                r.pos = endPos;
                return false;
            }
            node.type = "BONE";
            qint32 geosetId = -1;
            qint32 geoAnimId = -1;
            if (!r.readI32(geosetId) || !r.readI32(geoAnimId))
            {
                setErr(outError, "Unexpected EOF reading bone fields.");
                r.pos = endPos;
                return false;
            }
            Q_UNUSED(geosetId);
            Q_UNUSED(geoAnimId);
            storeNode(model, std::move(node));
        }
        r.pos = startPos + qsizetype(chunkSize);
        return true;
    }

    static bool parseHelpers(Reader& r, quint32 chunkSize, ModelData& model, QString* outError)
    {
        const qsizetype startPos = r.pos;
        const qsizetype endPos = startPos + qsizetype(chunkSize);
        while (r.pos + 4 <= endPos)
        {
            ModelData::Node node;
            if (!parseNodeBlock(r, node, outError))
            {
                r.pos = endPos;
                return false;
            }
            node.type = "HELP";
            storeNode(model, std::move(node));
        }
        r.pos = startPos + qsizetype(chunkSize);
        return true;
    }

    static bool parsePRE2(Reader& r, quint32 chunkSize, ModelData& model, QString* outError)
    {
        const qsizetype startPos = r.pos;
        const qsizetype endPos = startPos + qsizetype(chunkSize);
        while (r.pos + 4 <= endPos)
        {
            quint32 objectSize = 0;
            if (!r.readU32(objectSize)) break;
            if (objectSize < 8) break;
            const qsizetype objStart = r.pos - 4;
            const qsizetype objEnd = objStart + qsizetype(objectSize);
            if (objEnd > endPos) break;

            // Sub-reader scoped to this object
            Reader orr;
            orr.data = r.data + objStart;
            orr.size = qsizetype(objectSize);
            orr.pos = 4; // we already consumed objectSize

            ModelData::Node node;
            if (!parseNodeBlock(orr, node, outError))
            {
                r.pos = objEnd;
                continue;
            }
            node.type = "PRE2";

            ModelData::ParticleEmitter2 e;
            e.name = node.name;
            e.objectId = node.nodeId;
            e.parentId = node.parentId;
            e.flags = node.flags;
            storeNode(model, std::move(node));

            // Emitter2 fields
            if (!orr.readF32(e.speed) || !orr.readF32(e.variation) || !orr.readF32(e.latitude) ||
                !orr.readF32(e.gravity) || !orr.readF32(e.lifespan) || !orr.readF32(e.emissionRate) ||
                !orr.readF32(e.length) || !orr.readF32(e.width) || !orr.readU32(e.filterMode) ||
                !orr.readU32(e.rows) || !orr.readU32(e.columns) || !orr.readU32(e.headOrTail) ||
                !orr.readF32(e.tailLength) || !orr.readF32(e.timeMiddle))
            {
                r.pos = objEnd;
                continue;
            }

            for (int si = 0; si < 3; ++si)
            {
                if (!orr.readF32(e.segmentColor[si].x) || !orr.readF32(e.segmentColor[si].y) || !orr.readF32(e.segmentColor[si].z))
                { r.pos = objEnd; continue; }
            }
            if (!orr.readBytes(e.segmentAlpha, 3)) { r.pos = objEnd; continue; }
            for (int si = 0; si < 3; ++si)
                if (!orr.readF32(e.segmentScaling[si])) { r.pos = objEnd; continue; }

            for (int a = 0; a < 2; ++a)
                for (int b = 0; b < 3; ++b)
                    if (!orr.readU32(e.headIntervals[a][b])) { r.pos = objEnd; continue; }
            for (int a = 0; a < 2; ++a)
                for (int b = 0; b < 3; ++b)
                    if (!orr.readU32(e.tailIntervals[a][b])) { r.pos = objEnd; continue; }

            qint32 texId = -1;
            if (!orr.readI32(texId) || !orr.readU32(e.squirt) || !orr.readI32(e.priorityPlane) || !orr.readU32(e.replaceableId))
            {
                r.pos = objEnd;
                continue;
            }
            e.textureId = texId;

            // Parse remaining sub-chunks inside this object (animation tracks)
            while (orr.pos + 4 <= orr.size)
            {
                char t[4] = {};
                if (!orr.peekTag(t)) break;
                if (!std::isalpha((unsigned char)t[0])) break;
                if (!orr.readTag(t)) break;

                if (tagEq(t, "KP2S")) { if (!parseFloatTrack(orr, 0, e.trackSpeed)) break; }
                else if (tagEq(t, "KP2R")) { if (!parseFloatTrack(orr, 0, e.trackVariation)) break; }
                else if (tagEq(t, "KP2L")) { if (!parseFloatTrack(orr, 0, e.trackLatitude)) break; }
                else if (tagEq(t, "KP2G")) { if (!parseFloatTrack(orr, 0, e.trackGravity)) break; }
                else if (tagEq(t, "KP2E")) { if (!parseFloatTrack(orr, 0, e.trackEmissionRate)) break; }
                else if (tagEq(t, "KP2W")) { if (!parseFloatTrack(orr, 0, e.trackWidth)) break; }
                else if (tagEq(t, "KP2N")) { if (!parseFloatTrack(orr, 0, e.trackLength)) break; }
                else if (tagEq(t, "KP2V")) { if (!parseFloatTrack(orr, 0, e.trackVisibility)) break; }
                else
                {
                    LogSink::instance().log(QString("Unknown PRE2 track tag: %1%2%3%4")
                                                .arg(QChar(t[0])).arg(QChar(t[1]))
                                                .arg(QChar(t[2])).arg(QChar(t[3])));
                    break;
                }
            }

            model.emitters2.push_back(std::move(e));

            // Advance outer reader to end of object
            r.pos = objEnd;
        }

        // ensure r.pos at end of chunk
        r.pos = startPos + qsizetype(chunkSize);
        return true;
    }

    static bool parseNodeChunkObject(Reader& r, quint32 chunkSize, ModelData& model, QString* outError, const char* typeTag)
    {
        const qsizetype startPos = r.pos;
        const qsizetype endPos = startPos + qsizetype(chunkSize);
        while (r.pos + 4 <= endPos)
        {
            quint32 objectSize = 0;
            if (!r.readU32(objectSize)) break;
            if (objectSize < 8) break;
            const qsizetype objStart = r.pos - 4;
            const qsizetype objEnd = objStart + qsizetype(objectSize);
            if (objEnd > endPos) break;

            Reader orr;
            orr.data = r.data + objStart;
            orr.size = qsizetype(objectSize);
            orr.pos = 4;

            ModelData::Node node;
            if (parseNodeBlock(orr, node, outError))
            {
                if (typeTag)
                    node.type = typeTag;
                storeNode(model, std::move(node));
            }

            r.pos = objEnd;
        }
        r.pos = startPos + qsizetype(chunkSize);
        return true;
    }
}

namespace MdxLoader
{
    std::optional<ModelData> LoadFromBytes(const QByteArray& bytes, QString* outError)
    {
        if (bytes.size() < 8)
        {
            setErr(outError, "File too small.");
            return std::nullopt;
        }

        Reader r;
        r.data = reinterpret_cast<const unsigned char*>(bytes.constData());
        r.size = bytes.size();
        r.pos = 0;

        char magic[4] = {};
        if (!r.readTag(magic) || !tagEq(magic, "MDLX"))
        {
            setErr(outError, "Not an MDX file (missing MDLX magic).");
            return std::nullopt;
        }

        ModelData model;
        QStringList chunkTags;

        while (r.canRead(8))
        {
            char tag[4] = {};
            quint32 chunkSize = 0;
            if (!r.readTag(tag) || !r.readU32(chunkSize))
                break;
            if (!r.canRead(qsizetype(chunkSize)))
                break;

            const qsizetype chunkStart = r.pos;
            chunkTags << QString::fromLatin1(tag, 4);

            // Subreader for this chunk
            Reader cr;
            cr.data = r.data + chunkStart;
            cr.size = qsizetype(chunkSize);
            cr.pos = 0;

            if (tagEq(tag, "VERS"))
            {
                quint32 ver = 0;
                if (!cr.readU32(ver))
                {
                    setErr(outError, "Failed reading VERS.");
                    return std::nullopt;
                }
                model.mdxVersion = ver;
            }
            else if (tagEq(tag, "TEXS"))
            {
                if (!parseTextures(cr, chunkSize, model, outError))
                    return std::nullopt;
            }
            else if (tagEq(tag, "MTLS"))
            {
                if (!parseMaterials(cr, chunkSize, model.mdxVersion, model, outError))
                    return std::nullopt;
            }
            else if (tagEq(tag, "GEOS"))
            {
                // Parse geosets -> append vertices/indices and create submeshes
                std::uint32_t geosetIndex = 0;
                while (cr.canRead(4))
                {
                    quint32 inclusiveSize = 0;
                    if (!cr.readU32(inclusiveSize)) break;
                    if (inclusiveSize < 4)
                        break;

                    GeosetParsed gs;
                    if (!parseGeoset(cr, inclusiveSize, model.mdxVersion, gs, outError))
                        return std::nullopt;
                    model.geosetCount += 1;
                    LogSink::instance().log(QString("Geoset %1: verts=%2 tris=%3")
                                                .arg(geosetIndex)
                                                .arg(gs.vertices.size())
                                                .arg(gs.triIndices.size() / 3));
                    geosetIndex++;

                    ModelData::GeosetDiagnostics diag;
                    diag.gndx = std::move(gs.gndxRaw);
                    diag.mtgc = std::move(gs.mtgcRaw);
                    diag.mats = std::move(gs.matsRaw);
                    diag.expandedGroups = std::move(gs.expandedGroups);
                    model.geosetDiagnostics.push_back(std::move(diag));

                    if (gs.vertices.empty() || gs.triIndices.empty())
                        continue;

                    const std::uint32_t baseVertex = static_cast<std::uint32_t>(model.vertices.size());
                    const std::uint32_t indexOffset = static_cast<std::uint32_t>(model.indices.size());

                    // Append vertices
                    model.vertices.insert(model.vertices.end(), gs.vertices.begin(), gs.vertices.end());

                    if (!gs.vertexGroups.empty())
                    {
                        const std::size_t groupOffset = model.skinGroups.size();
                        for (auto& g : gs.groups)
                            model.skinGroups.push_back(std::move(g));

                        const std::size_t vgCount = std::min(gs.vertexGroups.size(), gs.vertices.size());
                        model.vertexGroups.reserve(model.vertexGroups.size() + gs.vertices.size());
                        for (std::size_t i = 0; i < vgCount; ++i)
                        {
                            const std::uint16_t gid = static_cast<std::uint16_t>(groupOffset + gs.vertexGroups[i]);
                            model.vertexGroups.push_back(gid);
                        }
                        for (std::size_t i = vgCount; i < gs.vertices.size(); ++i)
                        {
                            const std::uint16_t gid = static_cast<std::uint16_t>(groupOffset);
                            model.vertexGroups.push_back(gid);
                        }
                    }

                    // Append indices with baseVertex offset
                    for (std::uint32_t idx : gs.triIndices)
                        Q_ASSERT(idx < gs.vertices.size());
                    for (std::uint32_t idx : gs.triIndices)
                        model.indices.push_back(baseVertex + idx);

                    const std::uint32_t indexCount = static_cast<std::uint32_t>(model.indices.size()) - indexOffset;

                    SubMesh sm;
                    sm.indexOffset = indexOffset;
                    sm.indexCount = indexCount;
                    sm.materialId = gs.materialId;
                    model.subMeshes.push_back(sm);
                }
            }
            else if (tagEq(tag, "SEQS"))
            {
                if (!parseSequences(cr, chunkSize, model, outError))
                    return std::nullopt;
            }
            else if (tagEq(tag, "GLBS"))
            {
                if (!parseGlobalSequences(cr, chunkSize, model, outError))
                    return std::nullopt;
            }
            else if (tagEq(tag, "BONE"))
            {
                if (!parseBones(cr, chunkSize, model, outError))
                    return std::nullopt;
            }
            else if (tagEq(tag, "HELP"))
            {
                if (!parseHelpers(cr, chunkSize, model, outError))
                    return std::nullopt;
            }
            else if (tagEq(tag, "LITE") || tagEq(tag, "ATCH") || tagEq(tag, "PREM") ||
                     tagEq(tag, "RIBB") || tagEq(tag, "EVTS") || tagEq(tag, "CLID"))
            {
                char typeStr[5] = { tag[0], tag[1], tag[2], tag[3], 0 };
                if (!parseNodeChunkObject(cr, chunkSize, model, outError, typeStr))
                    return std::nullopt;
            }
            else if (tagEq(tag, "PIVT"))
            {
                if (!parsePivots(cr, chunkSize, model, outError))
                    return std::nullopt;
            }
            else if (tagEq(tag, "PRE2"))
            {
                if (!parsePRE2(cr, chunkSize, model, outError))
                    return std::nullopt;
            }

            // advance outer reader
            r.pos = chunkStart + qsizetype(chunkSize);
        }

        if (!chunkTags.isEmpty())
            LogSink::instance().log(QString("MDX chunks: %1").arg(chunkTags.join(", ")));

        if (!model.pivots.empty())
        {
            if (model.nodes.size() < model.pivots.size())
                model.nodes.resize(model.pivots.size());
            for (std::size_t i = 0; i < model.pivots.size(); ++i)
            {
                auto& n = model.nodes[i];
                if (n.nodeId < 0)
                    n.nodeId = static_cast<std::int32_t>(i);
                n.pivot = Vec3{model.pivots[i].x, model.pivots[i].y, model.pivots[i].z};
            }
        }

        model.bindVertices = model.vertices;

        const bool hasMesh = !model.vertices.empty() && !model.indices.empty();
        const bool hasParticles = !model.emitters2.empty();

        // Compute bounds (mesh preferred; otherwise use pivots; otherwise a small default cube)
        {
            float minv[3] = { std::numeric_limits<float>::infinity(),
                              std::numeric_limits<float>::infinity(),
                              std::numeric_limits<float>::infinity() };
            float maxv[3] = { -std::numeric_limits<float>::infinity(),
                              -std::numeric_limits<float>::infinity(),
                              -std::numeric_limits<float>::infinity() };

            if (hasMesh)
            {
                for (const auto& v : model.vertices)
                {
                    minv[0] = std::min(minv[0], v.px);
                    minv[1] = std::min(minv[1], v.py);
                    minv[2] = std::min(minv[2], v.pz);
                    maxv[0] = std::max(maxv[0], v.px);
                    maxv[1] = std::max(maxv[1], v.py);
                    maxv[2] = std::max(maxv[2], v.pz);
                }
            }
            else if (!model.pivots.empty())
            {
                for (const auto& p : model.pivots)
                {
                    minv[0] = std::min(minv[0], p.x);
                    minv[1] = std::min(minv[1], p.y);
                    minv[2] = std::min(minv[2], p.z);
                    maxv[0] = std::max(maxv[0], p.x);
                    maxv[1] = std::max(maxv[1], p.y);
                    maxv[2] = std::max(maxv[2], p.z);
                }
            }
            else
            {
                minv[0] = minv[1] = minv[2] = -1.0f;
                maxv[0] = maxv[1] = maxv[2] = 1.0f;
            }

            // Expand bounds slightly for particle-only models
            if (!hasMesh && hasParticles)
            {
                minv[0] -= 64.0f; minv[1] -= 64.0f; minv[2] -= 64.0f;
                maxv[0] += 64.0f; maxv[1] += 64.0f; maxv[2] += 64.0f;
            }

            model.boundsMin[0] = minv[0];
            model.boundsMin[1] = minv[1];
            model.boundsMin[2] = minv[2];
            model.boundsMax[0] = maxv[0];
            model.boundsMax[1] = maxv[1];
            model.boundsMax[2] = maxv[2];
            model.hasBounds = true;
        }

        // If MTLS chunk is missing, ensure at least 1 default material.
        if (model.materials.empty())
        {
            ModelMaterial m;
            m.priorityPlane = 0;
            m.flags = 0;
            m.layer.filterMode = 0;
            m.layer.shadingFlags = 0;
            m.layer.textureId = 0;
            m.layer.coordId = 0;
            m.layer.alpha = 1.0f;
            model.materials.push_back(m);
        }

        // Clamp material ids
        const std::uint32_t maxMat = static_cast<std::uint32_t>(model.materials.size());
        for (auto& sm : model.subMeshes)
        {
            if (sm.materialId >= maxMat)
                sm.materialId = 0;
        }

        return model;
    }

    std::optional<ModelData> LoadFromFile(const QString& filePath, QString* outError)
    {
        QFile f(filePath);
        if (!f.open(QIODevice::ReadOnly))
        {
            setErr(outError, QString("Failed to open: %1").arg(filePath));
            return std::nullopt;
        }
        const QByteArray bytes = f.readAll();
        auto model = LoadFromBytes(bytes, outError);
        if (!model)
            return std::nullopt;
        return model;
    }
}
