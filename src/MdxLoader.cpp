#include "MdxLoader.h"

#include <QFile>
#include <QtGlobal>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

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

    static quint32 primitiveIndexCount(quint32 d3dPrimitiveType, quint32 primitiveCount)
    {
        // Based on D3D-like primitive definitions used by Warcraft III MDX "PTYP"/"PCNT".
        switch (d3dPrimitiveType)
        {
        case 0:  return primitiveCount;           // point list
        case 1:  return primitiveCount * 2;       // line list
        case 2:  return primitiveCount + 1;       // line strip (approx)
        case 3:  return primitiveCount + 1;       // line loop (approx)
        case 4:  return primitiveCount * 3;       // triangle list
        case 5:  return primitiveCount + 2;       // triangle strip
        case 6:  return primitiveCount + 2;       // triangle fan
        case 7:  return primitiveCount * 4;       // quad list
        case 8:  return primitiveCount * 2 + 2;   // quad strip
        default: return 0;
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
        if (!gs.skip(qsizetype(vertexGroupCount)))
        {
            setErr(outError, "Unexpected EOF skipping GNDX.");
            return false;
        }

        // MTGC (skip)
        quint32 matrixGroupCount = 0;
        if (!readArrayTagCount(gs, "MTGC", matrixGroupCount, outError)) return false;
        if (!gs.skip(qsizetype(matrixGroupCount) * 4))
        {
            setErr(outError, "Unexpected EOF skipping MTGC.");
            return false;
        }

        // MATS (skip)
        quint32 matrixIndexCount = 0;
        if (!readArrayTagCount(gs, "MATS", matrixIndexCount, outError)) return false;
        if (!gs.skip(qsizetype(matrixIndexCount) * 4))
        {
            setErr(outError, "Unexpected EOF skipping MATS.");
            return false;
        }

        // Fixed fields
        quint32 materialId = 0, selectionGroup = 0, selectionFlags = 0;
        if (!gs.readU32(materialId) || !gs.readU32(selectionGroup) || !gs.readU32(selectionFlags))
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
                    out.vertices[i].v = v;
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
            const quint32 primCount = faceGroups[g];
            const quint32 idxCount = primitiveIndexCount(type, primCount);
            if (idxCount == 0) { cursor += idxCount; continue; }
            if (cursor + idxCount > faces.size()) break;
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
                if (layerSize < 28 || layerStart + qsizetype(layerSize) > mr.size)
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

                if (mdxVersion > 800)
                {
                    // layerShaderId
                    if (!mr.skip(4)) { setErr(outError, "Unexpected EOF skipping layerShaderId."); return false; }
                }

                quint32 textureId = 0, textureAnimId = 0, coordId = 0;
                float alpha = 1.0f, emissiveGain = 0.0f;
                if (!mr.readU32(textureId) || !mr.readU32(textureAnimId) || !mr.readU32(coordId) ||
                    !mr.readF32(alpha) || !mr.readF32(emissiveGain))
                {
                    setErr(outError, "Unexpected EOF reading layer fields.");
                    return false;
                }
                layer.textureId = textureId;
                layer.coordId = coordId;
                layer.alpha = alpha;

                // Skip remaining unknown tracks to end of layer
                mr.pos = layerStart + qsizetype(layerSize);

                if (li == 0)
                    m.layer = layer;
            }

            out.materials.push_back(m);
        }

        return true;
    }
}

namespace MdxLoader
{
    std::optional<ModelData> LoadFromFile(const QString& filePath, QString* outError)
    {
        QFile f(filePath);
        if (!f.open(QIODevice::ReadOnly))
        {
            setErr(outError, QString("Failed to open: %1").arg(filePath));
            return std::nullopt;
        }
        const QByteArray bytes = f.readAll();
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

        while (r.canRead(8))
        {
            char tag[4] = {};
            quint32 chunkSize = 0;
            if (!r.readTag(tag) || !r.readU32(chunkSize))
                break;
            if (!r.canRead(qsizetype(chunkSize)))
                break;

            const qsizetype chunkStart = r.pos;

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
                while (cr.canRead(4))
                {
                    quint32 inclusiveSize = 0;
                    if (!cr.readU32(inclusiveSize)) break;
                    if (inclusiveSize < 4)
                        break;

                    GeosetParsed gs;
                    if (!parseGeoset(cr, inclusiveSize, model.mdxVersion, gs, outError))
                        return std::nullopt;

                    if (gs.vertices.empty() || gs.triIndices.empty())
                        continue;

                    const std::uint32_t baseVertex = static_cast<std::uint32_t>(model.vertices.size());
                    const std::uint32_t indexOffset = static_cast<std::uint32_t>(model.indices.size());

                    // Append vertices
                    model.vertices.insert(model.vertices.end(), gs.vertices.begin(), gs.vertices.end());

                    // Append indices with baseVertex offset
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

            // advance outer reader
            r.pos = chunkStart + qsizetype(chunkSize);
        }

        if (model.vertices.empty() || model.indices.empty())
        {
            setErr(outError, "No geometry found (GEOS missing or empty).");
            return std::nullopt;
        }

        // Compute bounds
        float minv[3] = { std::numeric_limits<float>::infinity(),
                          std::numeric_limits<float>::infinity(),
                          std::numeric_limits<float>::infinity() };
        float maxv[3] = { -std::numeric_limits<float>::infinity(),
                          -std::numeric_limits<float>::infinity(),
                          -std::numeric_limits<float>::infinity() };

        for (const auto& v : model.vertices)
        {
            minv[0] = std::min(minv[0], v.px);
            minv[1] = std::min(minv[1], v.py);
            minv[2] = std::min(minv[2], v.pz);
            maxv[0] = std::max(maxv[0], v.px);
            maxv[1] = std::max(maxv[1], v.py);
            maxv[2] = std::max(maxv[2], v.pz);
        }
        model.boundsMin[0] = minv[0];
        model.boundsMin[1] = minv[1];
        model.boundsMin[2] = minv[2];
        model.boundsMax[0] = maxv[0];
        model.boundsMax[1] = maxv[1];
        model.boundsMax[2] = maxv[2];
        model.hasBounds = true;

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
}
