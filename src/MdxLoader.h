#pragma once

#include <optional>
#include <QString>
#include "ModelData.h"

// Minimal Warcraft III MDX (binary) loader.
// Focus: enough geometry/material data to render a static preview.

namespace MdxLoader
{
    // Loads an .mdx file from disk.
    // On failure returns std::nullopt and (optionally) fills outError.
    std::optional<ModelData> LoadFromFile(const QString& filePath, QString* outError = nullptr);
}
