#pragma once

#include <cstdint>
#include <string>
#include <vector>

// A minimal, static mesh representation of a Warcraft III MDX model.
// Goal: fast preview (static pose) with basic materials/textures.

struct ModelVertex
{
    float px = 0, py = 0, pz = 0;
    float nx = 0, ny = 0, nz = 1;
    float u = 0, v = 0;
};

struct ModelTexture
{
    std::uint32_t replaceableId = 0;
    std::string fileName; // e.g. "Textures\\Foo.blp"
    std::uint32_t flags = 0;
};

struct ModelLayer
{
    std::uint32_t filterMode = 0;    // see MDX spec (0..6)
    std::uint32_t shadingFlags = 0;  // bitfield
    std::uint32_t textureId = 0;     // index into ModelData::textures
    std::uint32_t coordId = 0;       // UV set (we currently use set 0 only)
    float alpha = 1.0f;
};

struct ModelMaterial
{
    int priorityPlane = 0;
    std::uint32_t flags = 0;
    // For preview we only use the first layer.
    ModelLayer layer;
};

struct SubMesh
{
    std::uint32_t indexOffset = 0; // into ModelData::indices
    std::uint32_t indexCount = 0;
    std::uint32_t materialId = 0; // index into ModelData::materials
};

// Small helper vector type used by the particle system.
struct Vec3 { float x=0, y=0, z=0; };
struct Vec4 { float x=0, y=0, z=0, w=1; };

struct ModelData
{
    std::vector<ModelVertex> vertices;
    std::vector<ModelVertex> bindVertices;
    std::vector<std::uint32_t> indices; // triangle list
    std::vector<SubMesh> subMeshes;
    std::uint32_t geosetCount = 0;

    std::vector<ModelTexture> textures;
    std::vector<ModelMaterial> materials;

    float boundsMin[3] = {0, 0, 0};
    float boundsMax[3] = {0, 0, 0};
    bool hasBounds = false;

    std::uint32_t mdxVersion = 800; // from VERS

    // ---- Animation (SEQS/GLBS) ----
    struct Sequence
    {
        std::string name;
        std::uint32_t startMs = 0;
        std::uint32_t endMs = 0;
        std::uint32_t flags = 0;
        float moveSpeed = 0.0f;
    };

    std::vector<Sequence> sequences;
    std::vector<std::uint32_t> globalSequencesMs; // GLBS durations (ms)

    // ---- Pivots (PIVT) ----
    struct Pivot
    {
        float x = 0, y = 0, z = 0;
    };
    std::vector<Pivot> pivots;

    // ---- MDX animation track ----
    enum class MdxInterp : std::int32_t
    {
        None = 0,
        Linear = 1,
        Hermite = 2,
        Bezier = 3,
    };

    template<typename T>
    struct MdxTrackKey
    {
        std::uint32_t timeMs = 0;
        T value{};
        T inTan{};
        T outTan{};
    };

    template<typename T>
    struct MdxTrack
    {
        MdxInterp interp = MdxInterp::None;
        std::int32_t globalSeqId = -1;
        std::vector<MdxTrackKey<T>> keys;
        bool empty() const { return keys.empty(); }
    };

    // ---- Nodes (BONE/HELP/...) ----
    struct Node
    {
        std::string name;
        std::string type; // e.g. BONE/HELP/ATCH/CLID/PRE2
        std::int32_t nodeId = -1;
        std::int32_t parentId = -1;
        std::uint32_t flags = 0;
        Vec3 pivot;
        MdxTrack<Vec3> trackTranslation;
        MdxTrack<Vec4> trackRotation;
        MdxTrack<Vec3> trackScaling;
    };

    std::vector<Node> nodes; // indexed by nodeId when possible
    std::vector<std::int32_t> boneNodeIds; // BONE chunk order -> nodeId

    struct SkinGroup
    {
        std::vector<std::int32_t> nodeIndices;
    };
    std::vector<std::uint16_t> vertexGroups; // per-vertex group id (index into skinGroups)
    std::vector<SkinGroup> skinGroups;

    struct GeosetDiagnostics
    {
        std::vector<std::uint8_t> gndx;
        std::vector<std::uint32_t> mtgc;
        std::vector<std::int32_t> mats;
        std::vector<std::vector<std::int32_t>> expandedGroups;
        std::uint32_t materialId = 0;
        std::uint32_t vertexCount = 0;
        std::uint32_t triCount = 0;
        std::uint32_t maxVertexGroup = 0;
        std::uint32_t baseVertex = 0;
        std::uint32_t indexOffset = 0;
        std::uint32_t indexCount = 0;
    };
    std::vector<GeosetDiagnostics> geosetDiagnostics;

    struct ParticleEmitter2
    {
        std::string name;
        std::int32_t objectId = -1;  // often indexes pivot
        std::int32_t parentId = -1;
        std::uint32_t flags = 0;

        float speed = 0.0f;
        float variation = 0.0f;
        float latitude = 0.0f;
        float gravity = 0.0f;
        float lifespan = 0.0f;
        float emissionRate = 0.0f;
        float width = 0.0f;
        float length = 0.0f;
        std::uint32_t filterMode = 0;
        std::uint32_t rows = 1;
        std::uint32_t columns = 1;
        std::uint32_t headOrTail = 0; // 0=head, 1=tail
        float tailLength = 0.0f;
        float timeMiddle = 0.5f;
        Vec3 segmentColor[3] = { {1,1,1},{1,1,1},{1,1,1} };
        std::uint8_t segmentAlpha[3] = {255,255,255};
        float segmentScaling[3] = {100,100,100}; // percent
        std::uint32_t headIntervals[2][3] = {};
        std::uint32_t tailIntervals[2][3] = {};
        std::int32_t textureId = -1;
        std::uint32_t squirt = 0;
        std::int32_t priorityPlane = 0;
        std::uint32_t replaceableId = 0;

        // Tracks (KP2*)
        MdxTrack<float> trackSpeed;        // KP2S
        MdxTrack<float> trackEmissionRate; // KP2E
        MdxTrack<float> trackGravity;      // KP2G
        MdxTrack<float> trackLifespan;     // KP2L
        MdxTrack<float> trackVisibility;   // KP2V
        MdxTrack<float> trackVariation;    // KP2R
        MdxTrack<float> trackLatitude;     // KP2L (rare)
        MdxTrack<float> trackWidth;        // KP2W
        MdxTrack<float> trackLength;       // KP2N
    };

    std::vector<ParticleEmitter2> emitters2;
};
