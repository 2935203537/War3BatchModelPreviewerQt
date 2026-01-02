#pragma once
#include <QString>
#include <QImage>
#include <QByteArray>

// Minimal Warcraft III BLP loader used for preview.
// Supports:
// - BLP1 / BLP2 headers
// - CONTENT_DIRECT palettized images with 0/1/4/8-bit alpha
// - CONTENT_JPEG: best-effort (may fail on some files)
// Returns a QImage in Format_RGBA8888.

namespace BlpLoader
{
    bool LoadBlpToImage(const QString& filePath, QImage* outImage, QString* outError = nullptr);
    bool LoadBlpToImageCached(const QString& filePath, QImage* outImage, QString* outError = nullptr);
    bool LoadBlpToImageFromBytes(const QByteArray& bytes, QImage* outImage, QString* outError = nullptr);
    inline bool LoadFromBytes(const QByteArray& bytes, QImage* outImage, QString* outError = nullptr)
    {
        return LoadBlpToImageFromBytes(bytes, outImage, outError);
    }
}
