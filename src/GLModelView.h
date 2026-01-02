#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLDebugLogger>
#include <QOpenGLShaderProgram>
#include <QVector3D>
#include <QMatrix4x4>
#include <QRandomGenerator>
#include <QTimer>
#include <QElapsedTimer>
#include <QHash>
#include <optional>
#include <unordered_map>

#include "ModelData.h"

class GLModelView final : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
    Q_OBJECT
public:
    explicit GLModelView(QWidget* parent = nullptr);
    ~GLModelView() override;

    void setModel(std::optional<ModelData> model, const QString& displayName, const QString& filePath);
    void setAssetRoot(const QString& assetRoot);
    void resetView();

    // Animation / playback
    // Default = 1.0; clamped to [0.05, 10.0]
    void setPlaybackSpeed(float speed);

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
    struct TextureResolve
    {
        QString path;
        QString source;
    };
    TextureResolve resolveTexturePath(const std::string& mdxPath) const;
    GLuint createPlaceholderTexture();

    struct GpuSubmesh
    {
        std::uint32_t indexOffset = 0;
        std::uint32_t indexCount = 0;
        std::uint32_t materialId = 0;
    };

    struct GpuCacheEntry
    {
        GLuint vao = 0;
        GLuint vbo = 0;
        GLuint ibo = 0;
        std::vector<GpuSubmesh> submeshes;
    };

    struct TextureHandle
    {
        GLuint id = 0;
        bool valid = false;
        QString path;
        QString source;
    };

    std::optional<ModelData> model_;
    QString displayName_;
    QString modelPath_;
    QString modelDir_;
    QString assetRoot_;

    // GPU resources
    GLuint vao_ = 0;
    GLuint vbo_ = 0;
    GLuint ibo_ = 0;
    std::vector<GpuSubmesh> gpuSubmeshes_;
    QHash<QString, GpuCacheEntry> gpuCache_;

    QOpenGLShaderProgram program_;
    bool programReady_ = false;
    QOpenGLDebugLogger glLogger_;
    bool glLoggerReady_ = false;

    std::unordered_map<std::uint32_t, TextureHandle> textureCache_;
    GLuint placeholderTex_ = 0;
    GLuint teamColorTex_ = 0;
    GLuint teamGlowTex_ = 0;

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

    // ---- Animation state ----
    float playbackSpeed_ = 1.0f;
    std::uint32_t localTimeMs_ = 0;
    int currentSeq_ = 0; // auto-play sequences[0]

    QTimer frameTick_;
    QElapsedTimer frameTimer_;

    void tickAnimation();
    void updateEmitters(float dtSeconds);

    // ---- Particle runtime ----
    struct Particle
    {
        QVector3D pos;
        QVector3D vel;
        float age = 0.0f;
        float life = 1.0f;
    };

    struct RuntimeEmitter2
    {
        double spawnAccum = 0.0;
        std::vector<Particle> particles;
    };

    std::vector<RuntimeEmitter2> runtimeEmitters2_;

    struct ParticleVertex
    {
        float px, py, pz;
        float u, v;
        float r, g, b, a;
    };
    std::vector<ParticleVertex> particleVerts_;

    QOpenGLShaderProgram particleProgram_;
    bool particleProgramReady_ = false;
    GLuint pVao_ = 0;
    GLuint pVbo_ = 0;
};
