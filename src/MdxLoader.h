#pragma once
#include <QString>
#include <optional>

#include "ModelData.h"

// Minimal Warcraft III MDX mesh loader.
// - Loads the first geoset's vertices and triangle indices.
// - Attempts to read normals; otherwise computes normals.
// - Ignores textures/materials/bones/animations.

class MdxLoader
{
public:
    static std::optional<ModelData> LoadFromFile(const QString& filePath, QString* outError);
};
