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
#include <QSet>
#include <memory>
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
    void setVfs(const std::shared_ptr<class IVfs>& vfs);
    void resetView();
    void setBackgroundAlpha(float alpha);
    void setCameraAngles(float yaw, float pitch, float roll);
    void setCameraPan(float x, float y, float z);
    void dumpCpuSkinCheck(const QString& outPath, int geosetIndex = 0);

    // Animation / playback
    // Default = 1.0; clamped to [0.05, 10.0]
    void setPlaybackSpeed(float speed);

signals:
    void statusTextChanged(const QString& text);
    void missingTexturesChanged(const QStringList& missing);
    void anglesChanged(float yaw, float pitch, float roll);
    void panChanged(float x, float y, float z);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;
    void mouseDoubleClickEvent(QMouseEvent* e) override;
    void keyPressEvent(QKeyEvent* e) override;

private:
    void rebuildGpuBuffers();
    void clearGpuResources();
    void computeModelBounds();
    void buildDebugGeometry();
    void updateProjection(int w, int h);
    void updateStatusText();
    void recordMissingTexture(const QString& ref, const QStringList& attempts);
    void drawDebug(const QMatrix4x4& mvp);
    void setGlPhase(const char* phase);
    void updateSkinning(std::uint32_t globalTimeMs);
    void buildNodeWorldCached(std::uint32_t globalTimeMs);

    GLuint getOrCreateTexture(std::uint32_t textureId);
    struct TextureResolve
    {
        QString path;
        QString source;
        QStringList attempts;
        QStringList vfsCandidates;
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
    std::shared_ptr<class IVfs> vfs_;

    // GPU resources
    GLuint vao_ = 0;
    GLuint vbo_ = 0;
    GLuint ibo_ = 0;
    std::vector<GpuSubmesh> gpuSubmeshes_;
    QHash<QString, GpuCacheEntry> gpuCache_;
    std::vector<ModelVertex> skinnedVertices_;

    QOpenGLShaderProgram program_;
    bool programReady_ = false;
    QOpenGLDebugLogger glLogger_;
    bool glLoggerReady_ = false;
    QString glPhase_;
    QOpenGLShaderProgram debugProgram_;
    bool debugProgramReady_ = false;
    GLuint debugVao_ = 0;
    GLuint debugVbo_ = 0;
    GLuint sanityVao_ = 0;
    GLuint sanityVbo_ = 0;
    struct DebugVertex
    {
        float px, py, pz;
        float r, g, b, a;
    };
    std::vector<DebugVertex> debugVerts_;

    std::unordered_map<std::uint32_t, TextureHandle> textureCache_;
    GLuint placeholderTex_ = 0;
    GLuint teamColorTex_ = 0;
    GLuint teamGlowTex_ = 0;
    QStringList missingTextures_;
    QSet<QString> missingTextureSet_;

    // Camera controls
    QPoint lastMouse_;
    float yaw_ = 30.0f;
    float pitch_ = -25.0f;
    float roll_ = 0.0f;
    float distance_ = 6.0f;
    float near_ = 0.05f;
    float far_ = 2000.0f;
    QVector3D panOffset_{0,0,0};
    bool wireframe_ = false;
    bool alphaTestEnabled_ = false;
    bool isGles_ = false;
    float backgroundAlpha_ = 1.0f;

    // Model framing
    QVector3D modelCenter_{0,0,0};
    float modelRadius_ = 1.0f;
    QVector3D boundsMin_{0,0,0};
    QVector3D boundsMax_{0,0,0};
    float boundsRadius_ = 1.0f;

    // Cached matrices
    QMatrix4x4 proj_;
    int viewportW_ = 1;
    int viewportH_ = 1;

    // Persistent node transforms (for DontInheritTranslation logic)
    std::vector<QMatrix4x4> nodeWorldMat_;
    std::vector<QVector3D> nodeWorldLoc_;
    std::vector<QQuaternion> nodeWorldRot_;
    std::vector<QVector3D> nodeWorldScale_;
    std::vector<QVector3D> nodeInvWorldLoc_;
    std::vector<QQuaternion> nodeInvWorldRot_;
    std::vector<QVector3D> nodeInvWorldScale_;

    // ---- Animation state ----
    float playbackSpeed_ = 1.0f;
    std::uint32_t localTimeMs_ = 0;
    std::uint32_t lastGlobalTimeMs_ = 0;
    int currentSeq_ = 0; // auto-play sequences[0]

    QTimer frameTick_;
    QElapsedTimer frameTimer_;
    QElapsedTimer fpsTimer_;
    int fpsFrames_ = 0;
    float fps_ = 0.0f;
    QElapsedTimer statusTimer_;
    int lastDrawCalls_ = 0;
    bool loggedBlank_ = false;

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
