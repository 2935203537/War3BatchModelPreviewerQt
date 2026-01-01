#pragma once
#include <vector>
#include <cstdint>

struct ModelVertex
{
    float px = 0, py = 0, pz = 0;
    float nx = 0, ny = 0, nz = 1;
};

struct ModelData
{
    std::vector<ModelVertex> vertices;
    std::vector<std::uint32_t> indices; // triangles (3 indices per triangle)

    float boundsMin[3] = {0, 0, 0};
    float boundsMax[3] = {0, 0, 0};
    bool hasBounds = false;
};
