#pragma once

#include <optional>
#include <QString>
#include <QByteArray>
#include "ModelData.h"

// Minimal Warcraft III MDX (binary) loader.
// Focus: enough geometry/material data to render a static preview.

namespace MdxLoader
{
    // Loads an .mdx file from disk.
    // On failure returns std::nullopt and (optionally) fills outError.
    std::optional<ModelData> LoadFromFile(const QString& filePath, QString* outError = nullptr);
    // Loads an .mdx file from memory bytes.
    std::optional<ModelData> LoadFromBytes(const QByteArray& bytes, QString* outError = nullptr);
}
