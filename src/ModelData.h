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

struct ModelData
{
    std::vector<ModelVertex> vertices;
    std::vector<std::uint32_t> indices; // triangle list
    std::vector<SubMesh> subMeshes;

    std::vector<ModelTexture> textures;
    std::vector<ModelMaterial> materials;

    float boundsMin[3] = {0, 0, 0};
    float boundsMax[3] = {0, 0, 0};
    bool hasBounds = false;

    std::uint32_t mdxVersion = 800; // from VERS
};
