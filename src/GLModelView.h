#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QVector3D>
#include <QMatrix4x4>
#include <optional>
#include <unordered_map>

#include "ModelData.h"

class GLModelView final : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
    Q_OBJECT
public:
    explicit GLModelView(QWidget* parent = nullptr);
    ~GLModelView() override;

    void setModel(std::optional<ModelData> model, const QString& displayName);
    void setAssetRoot(const QString& assetRoot);

signals:
    void statusTextChanged(const QString& text);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;

private:
    void rebuildGpuBuffers();
    void clearGpuResources();

    GLuint getOrCreateTexture(std::uint32_t textureId);
    QString resolveTexturePath(const std::string& mdxPath) const;
    GLuint createPlaceholderTexture();

    struct GpuSubmesh
    {
        std::uint32_t indexOffset = 0;
        std::uint32_t indexCount = 0;
        std::uint32_t materialId = 0;
    };

    struct TextureHandle
    {
        GLuint id = 0;
        bool valid = false;
    };

    std::optional<ModelData> model_;
    QString displayName_;
    QString assetRoot_;

    // GPU resources
    GLuint vao_ = 0;
    GLuint vbo_ = 0;
    GLuint ibo_ = 0;
    std::vector<GpuSubmesh> gpuSubmeshes_;

    QOpenGLShaderProgram program_;
    bool programReady_ = false;

    std::unordered_map<std::uint32_t, TextureHandle> textureCache_;
    GLuint placeholderTex_ = 0;

    // Camera controls
    QPoint lastMouse_;
    float yaw_ = 30.0f;
    float pitch_ = -25.0f;
    float distance_ = 6.0f;

    // Model framing
    QVector3D modelCenter_{0,0,0};
    float modelRadius_ = 1.0f;

    // Cached matrices
    QMatrix4x4 proj_;
};
