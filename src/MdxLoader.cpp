#include "MdxLoader.h"

#include <QFile>
#include <QByteArray>
#include <QtGlobal>

#include <cstring>
#include <cstdint>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>

namespace
{
    struct Reader
    {
        const std::uint8_t* p = nullptr;
        const std::uint8_t* end = nullptr;

        bool ok() const { return p && p <= end; }
        std::size_t remaining() const { return (p <= end) ? static_cast<std::size_t>(end - p) : 0; }

        bool skip(std::size_t n)
        {
            if (remaining() < n) return false;
            p += n;
            return true;
        }

        template<typename T>
        bool read(T& out)
        {
            if (remaining() < sizeof(T)) return false;
            std::memcpy(&out, p, sizeof(T));
            p += sizeof(T);
            return true;
        }

        bool readBytes(void* out, std::size_t n)
        {
            if (remaining() < n) return false;
            std::memcpy(out, p, n);
            p += n;
            return true;
        }

        bool peekTag(char out[4]) const
        {
            if (remaining() < 4) return false;
            std::memcpy(out, p, 4);
            return true;
        }
    };

    static bool IsLikelyChunkTag(const char t[4])
    {
        // MDX tags are typically uppercase ASCII letters (sometimes digits).
        for (int i = 0; i < 4; ++i)
        {
            unsigned char c = static_cast<unsigned char>(t[i]);
            const bool isUpper = (c >= 'A' && c <= 'Z');
            const bool isDigit = (c >= '0' && c <= '9');
            if (!(isUpper || isDigit)) return false;
        }
        return true;
    }

    static std::uint32_t ReadU32(const std::uint8_t* p)
    {
        std::uint32_t v;
        std::memcpy(&v, p, sizeof(v));
        return v;
    }

    static void ComputeBounds(ModelData& m)
    {
        if (m.vertices.empty()) return;
        float mn[3] = { m.vertices[0].px, m.vertices[0].py, m.vertices[0].pz };
        float mx[3] = { m.vertices[0].px, m.vertices[0].py, m.vertices[0].pz };
        for (const auto& v : m.vertices)
        {
            mn[0] = std::min(mn[0], v.px); mn[1] = std::min(mn[1], v.py); mn[2] = std::min(mn[2], v.pz);
            mx[0] = std::max(mx[0], v.px); mx[1] = std::max(mx[1], v.py); mx[2] = std::max(mx[2], v.pz);
        }
        m.boundsMin[0] = mn[0]; m.boundsMin[1] = mn[1]; m.boundsMin[2] = mn[2];
        m.boundsMax[0] = mx[0]; m.boundsMax[1] = mx[1]; m.boundsMax[2] = mx[2];
        m.hasBounds = true;
    }

    static void Normalize3(float& x, float& y, float& z)
    {
        const float len = std::sqrt(x*x + y*y + z*z);
        if (len > 1e-12f)
        {
            x /= len; y /= len; z /= len;
        }
    }

    static void ComputeNormalsIfMissing(ModelData& m, bool hadNormals)
    {
        if (hadNormals) return;
        for (auto& v : m.vertices)
        {
            v.nx = v.ny = 0.0f; v.nz = 0.0f;
        }

        for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3)
        {
            const std::uint32_t i0 = m.indices[i + 0];
            const std::uint32_t i1 = m.indices[i + 1];
            const std::uint32_t i2 = m.indices[i + 2];
            if (i0 >= m.vertices.size() || i1 >= m.vertices.size() || i2 >= m.vertices.size()) continue;

            const auto& v0 = m.vertices[i0];
            const auto& v1 = m.vertices[i1];
            const auto& v2 = m.vertices[i2];

            const float ax = v1.px - v0.px;
            const float ay = v1.py - v0.py;
            const float az = v1.pz - v0.pz;

            const float bx = v2.px - v0.px;
            const float by = v2.py - v0.py;
            const float bz = v2.pz - v0.pz;

            float nx = ay*bz - az*by;
            float ny = az*bx - ax*bz;
            float nz = ax*by - ay*bx;

            m.vertices[i0].nx += nx; m.vertices[i0].ny += ny; m.vertices[i0].nz += nz;
            m.vertices[i1].nx += nx; m.vertices[i1].ny += ny; m.vertices[i1].nz += nz;
            m.vertices[i2].nx += nx; m.vertices[i2].ny += ny; m.vertices[i2].nz += nz;
        }

        for (auto& v : m.vertices)
        {
            Normalize3(v.nx, v.ny, v.nz);
        }
    }

    // Parse GEOS chunk and load the first geoset's VRTX/NRMS/PTYP/PCNT/PVTX.
    static bool ParseGeosChunk(Reader& r, std::size_t chunkSize, ModelData& out, QString* outError)
    {
        const std::uint8_t* chunkEnd = r.p + chunkSize;
        if (chunkEnd > r.end) return false;

        bool loadedAny = false;

        // GEOS contains multiple geosets. Each geoset begins with a uint32 inclusiveSize.
        while (r.p + 4 <= chunkEnd)
        {
            // Need at least 4 bytes for geoset size
            std::uint32_t geosetSize = 0;
            if (!r.read(geosetSize)) break;
            if (geosetSize < 4 || r.p + (geosetSize - 4) > chunkEnd)
            {
                // If this doesn't look like a geoset, bail out safely.
                break;
            }

            Reader gr;
            gr.p = r.p;
            gr.end = r.p + (geosetSize - 4); // body bytes
            r.p = gr.end; // advance outer reader to end of this geoset body

            std::vector<float> verts; // xyz packed
            std::vector<float> norms; // xyz packed
            std::vector<std::uint32_t> ptyp;
            std::vector<std::uint32_t> pcnt;
            std::vector<std::uint16_t> pvtx;

            bool gotNormals = false;

            while (gr.remaining() >= 8)
            {
                char tag[4];
                if (!gr.peekTag(tag)) break;
                if (!IsLikelyChunkTag(tag))
                {
                    // We've likely reached the fixed tail fields of the geoset.
                    break;
                }

                // read tag + size
                gr.skip(4);
                std::uint32_t subSize = 0;
                if (!gr.read(subSize)) break;
                if (gr.remaining() < subSize) break;

                const std::uint8_t* subData = gr.p;
                gr.skip(subSize);

                auto tagStr = std::string(tag, tag + 4);

                if (tagStr == "VRTX")
                {
                    const std::size_t count = subSize / (sizeof(float) * 3);
                    verts.resize(count * 3);
                    std::memcpy(verts.data(), subData, count * 3 * sizeof(float));
                }
                else if (tagStr == "NRMS")
                {
                    const std::size_t count = subSize / (sizeof(float) * 3);
                    norms.resize(count * 3);
                    std::memcpy(norms.data(), subData, count * 3 * sizeof(float));
                    gotNormals = true;
                }
                else if (tagStr == "PTYP")
                {
                    const std::size_t count = subSize / sizeof(std::uint32_t);
                    ptyp.resize(count);
                    std::memcpy(ptyp.data(), subData, count * sizeof(std::uint32_t));
                }
                else if (tagStr == "PCNT")
                {
                    const std::size_t count = subSize / sizeof(std::uint32_t);
                    pcnt.resize(count);
                    std::memcpy(pcnt.data(), subData, count * sizeof(std::uint32_t));
                }
                else if (tagStr == "PVTX")
                {
                    const std::size_t count = subSize / sizeof(std::uint16_t);
                    pvtx.resize(count);
                    std::memcpy(pvtx.data(), subData, count * sizeof(std::uint16_t));
                }
                // ignore other chunks
            }

            if (!verts.empty() && !pvtx.empty() && !ptyp.empty() && !pcnt.empty())
            {
                // Build ModelData from first geoset
                const std::size_t vcount = verts.size() / 3;
                out.vertices.resize(vcount);
                for (std::size_t i = 0; i < vcount; ++i)
                {
                    out.vertices[i].px = verts[i*3 + 0];
                    out.vertices[i].py = verts[i*3 + 1];
                    out.vertices[i].pz = verts[i*3 + 2];

                    if (gotNormals && norms.size() >= (i*3 + 3))
                    {
                        out.vertices[i].nx = norms[i*3 + 0];
                        out.vertices[i].ny = norms[i*3 + 1];
                        out.vertices[i].nz = norms[i*3 + 2];
                    }
                }

                // Convert primitives to triangle index list (supports type 4=triangles).
                std::size_t cursor = 0;
                for (std::size_t pi = 0; pi < ptyp.size() && pi < pcnt.size(); ++pi)
                {
                    const std::uint32_t type = ptyp[pi];
                    const std::uint32_t count = pcnt[pi];

                    if (type == 4) // triangles
                    {
                        const std::size_t need = static_cast<std::size_t>(count) * 3;
                        if (cursor + need > pvtx.size()) break;
                        for (std::size_t k = 0; k < need; ++k)
                        {
                            out.indices.push_back(static_cast<std::uint32_t>(pvtx[cursor + k]));
                        }
                        cursor += need;
                    }
                    else
                    {
                        // Skip unsupported primitives conservatively.
                        // Many MDX models only use triangles.
                        // We still need to advance cursor:
                        // - A safe approach is to stop here (avoid desync).
                        break;
                    }
                }

                if (out.indices.size() >= 3)
                {
                    ComputeBounds(out);
                    ComputeNormalsIfMissing(out, gotNormals);
                    loadedAny = true;
                    break; // first geoset only
                }
            }
        }

        if (!loadedAny)
        {
            if (outError) *outError = "No supported GEOS/Geoset mesh data found (need VRTX/PTYP/PCNT/PVTX).";
        }

        // move reader to end of GEOS chunk
        r.p = chunkEnd;
        return loadedAny;
    }
}

std::optional<ModelData> MdxLoader::LoadFromFile(const QString& filePath, QString* outError)
{
    QFile f(filePath);
    if (!f.open(QIODevice::ReadOnly))
    {
        if (outError) *outError = "Failed to open file.";
        return std::nullopt;
    }

    QByteArray bytes = f.readAll();
    if (bytes.size() < 8)
    {
        if (outError) *outError = "File too small.";
        return std::nullopt;
    }

    Reader r;
    r.p = reinterpret_cast<const std::uint8_t*>(bytes.constData());
    r.end = r.p + bytes.size();

    char magic[4];
    if (!r.readBytes(magic, 4))
    {
        if (outError) *outError = "Failed to read header.";
        return std::nullopt;
    }

    if (std::memcmp(magic, "MDLX", 4) != 0)
    {
        if (outError) *outError = "Not an MDX file (missing MDLX header).";
        return std::nullopt;
    }

    ModelData model;
    bool gotMesh = false;

    // Parse top-level chunks: [tag(4)][size(u32)][data...]
    while (r.remaining() >= 8)
    {
        char tag[4];
        if (!r.readBytes(tag, 4)) break;

        std::uint32_t size = 0;
        if (!r.read(size)) break;
        if (r.remaining() < size) break;

        auto tagStr = std::string(tag, tag + 4);

        if (tagStr == "GEOS")
        {
            Reader geosR;
            geosR.p = r.p;
            geosR.end = r.p + size;
            QString err;
            ModelData tmp;
            if (ParseGeosChunk(geosR, size, tmp, &err))
            {
                model = std::move(tmp);
                gotMesh = true;
            }
            // advance outer reader regardless
            r.skip(size);
        }
        else
        {
            r.skip(size);
        }

        if (gotMesh) break;
    }

    if (!gotMesh)
    {
        if (outError && outError->isEmpty()) *outError = "Failed to find GEOS mesh chunk.";
        return std::nullopt;
    }

    return model;
}
