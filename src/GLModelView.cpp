#include "GLModelView.h"

#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QFileInfo>
#include <QDir>
#include <QDirIterator>
#include <QImage>
#include <QOpenGLContext>
#include <QQuaternion>
#include <QVector4D>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <random>

#include "BlpLoader.h"
#include "LogSink.h"
#include "Vfs.h"

namespace
{
    using MdxInterp = ModelData::MdxInterp;

    template<typename T>
    using MdxTrack = ModelData::MdxTrack<T>;

    static QString normPath(const QString& p)
    {
        QString s = p;
        s.replace('\\', '/');
        return s;
    }

    static float clampf(float v, float lo, float hi)
    {
        return (v < lo) ? lo : (v > hi) ? hi : v;
    }

    static float lerpf(float a, float b, float t)
    {
        return a + (b - a) * t;
    }

    static float hermite(float p0, float m0, float p1, float m1, float t)
    {
        const float t2 = t * t;
        const float t3 = t2 * t;
        return (2*t3 - 3*t2 + 1) * p0 +
               (t3 - 2*t2 + t) * m0 +
               (-2*t3 + 3*t2) * p1 +
               (t3 - t2) * m1;
    }

    static float bezier(float p0, float c1, float c2, float p1, float t)
    {
        const float it = 1.0f - t;
        return it*it*it*p0 + 3*it*it*t*c1 + 3*it*t*t*c2 + t*t*t*p1;
    }

    static Vec3 lerpVec3(const Vec3& a, const Vec3& b, float t)
    {
        return { lerpf(a.x, b.x, t), lerpf(a.y, b.y, t), lerpf(a.z, b.z, t) };
    }

    static Vec3 hermiteVec3(const Vec3& p0, const Vec3& m0, const Vec3& p1, const Vec3& m1, float t)
    {
        return {
            hermite(p0.x, m0.x, p1.x, m1.x, t),
            hermite(p0.y, m0.y, p1.y, m1.y, t),
            hermite(p0.z, m0.z, p1.z, m1.z, t)
        };
    }

    static Vec3 bezierVec3(const Vec3& p0, const Vec3& c1, const Vec3& c2, const Vec3& p1, float t)
    {
        return {
            bezier(p0.x, c1.x, c2.x, p1.x, t),
            bezier(p0.y, c1.y, c2.y, p1.y, t),
            bezier(p0.z, c1.z, c2.z, p1.z, t)
        };
    }

    static Vec4 lerpVec4(const Vec4& a, const Vec4& b, float t)
    {
        return { lerpf(a.x, b.x, t), lerpf(a.y, b.y, t), lerpf(a.z, b.z, t), lerpf(a.w, b.w, t) };
    }

    static Vec4 normalizeQuat(const Vec4& q)
    {
        const float len = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
        if (len <= 0.000001f)
            return {0,0,0,1};
        const float inv = 1.0f / len;
        return { q.x * inv, q.y * inv, q.z * inv, q.w * inv };
    }

    static float dotQuat(const Vec4& a, const Vec4& b)
    {
        return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
    }

    static Vec4 slerpQuat(Vec4 a, Vec4 b, float t, bool invertIfNecessary)
    {
        float dot = dotQuat(a, b);
        if (invertIfNecessary && dot < 0.0f)
        {
            dot = -dot;
            b.x = -b.x; b.y = -b.y; b.z = -b.z; b.w = -b.w;
        }

        if (dot > 0.95f)
        {
            return normalizeQuat(lerpVec4(a, b, t));
        }

        dot = clampf(dot, -1.0f, 1.0f);
        const float theta0 = std::acos(dot);
        const float sinTheta0 = std::sin(theta0);
        if (sinTheta0 <= 0.000001f)
            return normalizeQuat(lerpVec4(a, b, t));
        const float theta = theta0 * t;
        const float sinTheta = std::sin(theta);
        const float s0 = std::cos(theta) - dot * sinTheta / sinTheta0;
        const float s1 = sinTheta / sinTheta0;
        return { a.x * s0 + b.x * s1, a.y * s0 + b.y * s1, a.z * s0 + b.z * s1, a.w * s0 + b.w * s1 };
    }

    static float sampleTrackFloat(const MdxTrack<float>& tr, std::uint32_t timeMs, float def, const ModelData& model)
    {
        if (tr.keys.empty())
            return def;

        if (tr.globalSeqId >= 0 && std::size_t(tr.globalSeqId) < model.globalSequencesMs.size())
        {
            const std::uint32_t len = model.globalSequencesMs[std::size_t(tr.globalSeqId)];
            if (len != 0)
                timeMs = timeMs % len;
        }

        const auto& keys = tr.keys;
        if (timeMs <= keys.front().timeMs)
            return keys.front().value;
        if (timeMs >= keys.back().timeMs)
            return keys.back().value;

        // find segment
        std::size_t hi = 1;
        while (hi < keys.size() && timeMs > keys[hi].timeMs)
            ++hi;
        if (hi >= keys.size())
            return keys.back().value;
        const std::size_t lo = hi - 1;

        const auto& k0 = keys[lo];
        const auto& k1 = keys[hi];
        const float denom = float(k1.timeMs - k0.timeMs);
        const float t = denom > 0.0f ? float(timeMs - k0.timeMs) / denom : 0.0f;

        switch (tr.interp)
        {
        case MdxInterp::None:
            return k0.value;
        case MdxInterp::Linear:
            return lerpf(k0.value, k1.value, t);
        case MdxInterp::Hermite:
            // MDX stores tangents per key; use outTan of k0 and inTan of k1.
            return hermite(k0.value, k0.outTan, k1.value, k1.inTan, t);
        case MdxInterp::Bezier:
            // Treat tangents as Bezier control points.
            return bezier(k0.value, k0.outTan, k1.inTan, k1.value, t);
        default:
            return k0.value;
        }
    }

    static Vec3 sampleTrackVec3(const MdxTrack<Vec3>& tr, std::uint32_t timeMs, const Vec3& def, const ModelData& model)
    {
        if (tr.keys.empty())
            return def;

        if (tr.globalSeqId >= 0 && std::size_t(tr.globalSeqId) < model.globalSequencesMs.size())
        {
            const std::uint32_t len = model.globalSequencesMs[std::size_t(tr.globalSeqId)];
            if (len != 0)
                timeMs = timeMs % len;
        }

        const auto& keys = tr.keys;
        if (timeMs <= keys.front().timeMs)
            return keys.front().value;
        if (timeMs >= keys.back().timeMs)
            return keys.back().value;

        std::size_t hi = 1;
        while (hi < keys.size() && timeMs > keys[hi].timeMs)
            ++hi;
        if (hi >= keys.size())
            return keys.back().value;
        const std::size_t lo = hi - 1;

        const auto& k0 = keys[lo];
        const auto& k1 = keys[hi];
        const float denom = float(k1.timeMs - k0.timeMs);
        const float t = denom > 0.0f ? float(timeMs - k0.timeMs) / denom : 0.0f;

        switch (tr.interp)
        {
        case MdxInterp::None:
            return k0.value;
        case MdxInterp::Linear:
            return lerpVec3(k0.value, k1.value, t);
        case MdxInterp::Hermite:
            return hermiteVec3(k0.value, k0.outTan, k1.value, k1.inTan, t);
        case MdxInterp::Bezier:
            return bezierVec3(k0.value, k0.outTan, k1.inTan, k1.value, t);
        default:
            return k0.value;
        }
    }

    static Vec4 sampleTrackQuat(const MdxTrack<Vec4>& tr, std::uint32_t timeMs, const Vec4& def, const ModelData& model)
    {
        if (tr.keys.empty())
            return def;

        if (tr.globalSeqId >= 0 && std::size_t(tr.globalSeqId) < model.globalSequencesMs.size())
        {
            const std::uint32_t len = model.globalSequencesMs[std::size_t(tr.globalSeqId)];
            if (len != 0)
                timeMs = timeMs % len;
        }

        const auto& keys = tr.keys;
        if (timeMs <= keys.front().timeMs)
            return normalizeQuat(keys.front().value);
        if (timeMs >= keys.back().timeMs)
            return normalizeQuat(keys.back().value);

        std::size_t hi = 1;
        while (hi < keys.size() && timeMs > keys[hi].timeMs)
            ++hi;
        if (hi >= keys.size())
            return normalizeQuat(keys.back().value);
        const std::size_t lo = hi - 1;

        const auto& k0 = keys[lo];
        const auto& k1 = keys[hi];
        const float denom = float(k1.timeMs - k0.timeMs);
        const float t = denom > 0.0f ? float(timeMs - k0.timeMs) / denom : 0.0f;

        switch (tr.interp)
        {
        case MdxInterp::None:
            return normalizeQuat(k0.value);
        case MdxInterp::Linear:
            return slerpQuat(k0.value, k1.value, t, true);
        case MdxInterp::Hermite:
        {
            const Vec4 slerp = slerpQuat(k0.value, k1.value, t, false);
            const Vec4 slerpTan = slerpQuat(k0.outTan, k1.inTan, t, false);
            return slerpQuat(slerp, slerpTan, 2.0f * t * (1.0f - t), false);
        }
        case MdxInterp::Bezier:
        {
            const Vec4 s0 = slerpQuat(k0.value, k0.outTan, t, false);
            const Vec4 s1 = slerpQuat(k0.outTan, k1.inTan, t, false);
            const Vec4 s2 = slerpQuat(k1.inTan, k1.value, t, false);
            const Vec4 s3 = slerpQuat(s0, s1, t, false);
            const Vec4 s4 = slerpQuat(s1, s2, t, false);
            return slerpQuat(s3, s4, t, false);
        }
        default:
            return normalizeQuat(k0.value);
        }
    }

    // MDX Layer shading flags (common ones used for preview)
    constexpr std::uint32_t LAYER_UNSHADED   = 0x1;
    constexpr std::uint32_t LAYER_TWOSIDED   = 0x10;
    constexpr std::uint32_t LAYER_NODEPTH    = 0x40;
    constexpr std::uint32_t LAYER_NODEPTHSET = 0x80;

    constexpr std::uint32_t NODE_DONT_INHERIT_TRANSLATION = 0x1;
    constexpr std::uint32_t NODE_DONT_INHERIT_ROTATION    = 0x2;
    constexpr std::uint32_t NODE_DONT_INHERIT_SCALING     = 0x4;
}

GLModelView::GLModelView(QWidget* parent)
    : QOpenGLWidget(parent)
{
    setFocusPolicy(Qt::StrongFocus);

    // Drive animation with a timer (keeps particles alive even without input)
    connect(&frameTick_, &QTimer::timeout, this, &GLModelView::tickAnimation);
    frameTick_.setInterval(16); // ~60 FPS
    frameTick_.start();
    frameTimer_.start();
}

GLModelView::~GLModelView()
{
    makeCurrent();
    clearGpuResources();
    doneCurrent();
}

void GLModelView::setGlPhase(const char* phase)
{
    glPhase_ = QString::fromLatin1(phase);
}

void GLModelView::setPlaybackSpeed(float speed)
{
    playbackSpeed_ = clampf(speed, 0.05f, 10.0f);
}

void GLModelView::setBackgroundAlpha(float alpha)
{
    backgroundAlpha_ = clampf(alpha, 0.0f, 1.0f);
    update();
}

void GLModelView::setCameraAngles(float yaw, float pitch, float roll)
{
    yaw_ = yaw;
    pitch_ = clampf(pitch, -89.0f, 89.0f);
    roll_ = roll;
    emit anglesChanged(yaw_, pitch_, roll_);
    update();
}

void GLModelView::setCameraPan(float x, float y, float z)
{
    panOffset_ = QVector3D(x, y, z);
    emit panChanged(panOffset_.x(), panOffset_.y(), panOffset_.z());
    update();
}

void GLModelView::setAssetRoot(const QString& assetRoot)
{
    assetRoot_ = assetRoot;
}

void GLModelView::setVfs(const std::shared_ptr<IVfs>& vfs)
{
    vfs_ = vfs;
}

void GLModelView::resetView()
{
    // Default to a front-facing view (War3 uses Z-up).
    yaw_ = 0.0f;
    pitch_ = -90.0f;
    roll_ = -90.0f;
    distance_ = std::max(0.5f, modelRadius_ * 1.2f);
    panOffset_ = QVector3D(0, 0, 0);
    updateProjection(viewportW_, viewportH_);
    emit anglesChanged(yaw_, pitch_, roll_);
    emit panChanged(panOffset_.x(), panOffset_.y(), panOffset_.z());
    update();
}

void GLModelView::computeModelBounds()
{
    if (model_ && !model_->vertices.empty())
    {
        boundsMin_ = QVector3D(model_->vertices[0].px, model_->vertices[0].py, model_->vertices[0].pz);
        boundsMax_ = boundsMin_;
        for (const auto& v : model_->vertices)
        {
            boundsMin_.setX(std::min(boundsMin_.x(), v.px));
            boundsMin_.setY(std::min(boundsMin_.y(), v.py));
            boundsMin_.setZ(std::min(boundsMin_.z(), v.pz));
            boundsMax_.setX(std::max(boundsMax_.x(), v.px));
            boundsMax_.setY(std::max(boundsMax_.y(), v.py));
            boundsMax_.setZ(std::max(boundsMax_.z(), v.pz));
        }
    }
    else if (model_ && !model_->pivots.empty())
    {
        boundsMin_ = QVector3D(model_->pivots[0].x, model_->pivots[0].y, model_->pivots[0].z);
        boundsMax_ = boundsMin_;
        for (const auto& p : model_->pivots)
        {
            boundsMin_.setX(std::min(boundsMin_.x(), p.x));
            boundsMin_.setY(std::min(boundsMin_.y(), p.y));
            boundsMin_.setZ(std::min(boundsMin_.z(), p.z));
            boundsMax_.setX(std::max(boundsMax_.x(), p.x));
            boundsMax_.setY(std::max(boundsMax_.y(), p.y));
            boundsMax_.setZ(std::max(boundsMax_.z(), p.z));
        }
    }
    else
    {
        boundsMin_ = QVector3D(-1.0f, -1.0f, -1.0f);
        boundsMax_ = QVector3D(1.0f, 1.0f, 1.0f);
    }

    modelCenter_ = (boundsMin_ + boundsMax_) * 0.5f;
    const QVector3D ext = (boundsMax_ - boundsMin_) * 0.5f;
    boundsRadius_ = std::max(0.25f, ext.length());
    modelRadius_ = boundsRadius_;
    buildDebugGeometry();
}

void GLModelView::updateProjection(int w, int h)
{
    viewportW_ = std::max(w, 1);
    viewportH_ = std::max(h, 1);
    const float aspect = float(viewportW_) / float(viewportH_);
    near_ = std::max(0.05f, modelRadius_ / 5000.0f);
    far_ = std::max(2000.0f, distance_ + modelRadius_ * 10.0f);
    proj_.setToIdentity();
    proj_.perspective(45.0f, aspect, near_, far_);
}

void GLModelView::recordMissingTexture(const QString& ref, const QStringList& attempts)
{
    QString entry = ref;
    if (!attempts.isEmpty())
    {
        entry += "\n  tried:";
        for (const auto& a : attempts)
            entry += "\n    " + a;
    }
    if (missingTextureSet_.contains(entry))
        return;
    missingTextureSet_.insert(entry);
    missingTextures_.append(entry);
    emit missingTexturesChanged(missingTextures_);
}

void GLModelView::setModel(std::optional<ModelData> model, const QString& displayName, const QString& filePath)
{
    displayName_ = displayName;
    modelPath_ = filePath;
    modelDir_ = filePath.isEmpty() ? QString() : QFileInfo(filePath).absolutePath();
    model_ = std::move(model);
    skinnedVertices_.clear();

    missingTextures_.clear();
    missingTextureSet_.clear();
    emit missingTexturesChanged(missingTextures_);

    localTimeMs_ = 0;
    currentSeq_ = 0;
    frameTimer_.restart();
    fpsFrames_ = 0;
    fps_ = 0.0f;
    fpsTimer_.invalidate();
    loggedBlank_ = false;

    runtimeEmitters2_.clear();
    if (model_)
    {
        runtimeEmitters2_.resize(model_->emitters2.size());
    }

    // Reset texture cache (textures are tied to model/material IDs)
    if (context() && context()->isValid())
    {
        makeCurrent();
        textureCache_.clear();
        if (placeholderTex_ != 0)
        {
            glDeleteTextures(1, &placeholderTex_);
            placeholderTex_ = 0;
        }
        rebuildGpuBuffers();
        doneCurrent();
    }

    computeModelBounds();
    resetView();
    LogSink::instance().log(QString("Camera fit: target=%1,%2,%3 dist=%4 near=%5 far=%6")
                                .arg(modelCenter_.x()).arg(modelCenter_.y()).arg(modelCenter_.z())
                                .arg(distance_)
                                .arg(near_)
                                .arg(far_));

    QString seqInfo = "<no SEQS>";
    if (model_ && !model_->sequences.empty())
        seqInfo = QString("SEQ0=%1 [%2..%3]")
                      .arg(QString::fromStdString(model_->sequences[0].name))
                      .arg(model_->sequences[0].startMs)
                      .arg(model_->sequences[0].endMs);

    QString geoInfo = "<no geometry>";
    if (model_ && !model_->indices.empty())
    {
        geoInfo = QString("%1 verts, %2 tris, %3 submeshes")
                      .arg(model_->vertices.size())
                      .arg(model_->indices.size() / 3)
                      .arg(model_->subMeshes.size());
    }
    QString fxInfo = "<no PRE2>";
    if (model_ && !model_->emitters2.empty())
        fxInfo = QString("PRE2=%1").arg(model_->emitters2.size());

    emit statusTextChanged(QString("%1 | %2 | %3 | %4")
                               .arg(displayName_)
                               .arg(geoInfo)
                               .arg(fxInfo)
                               .arg(seqInfo));

    update();
}

void GLModelView::initializeGL()
{
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);

    if (glLogger_.initialize())
    {
        glLoggerReady_ = true;
        connect(&glLogger_, &QOpenGLDebugLogger::messageLogged, this,
                [this](const QOpenGLDebugMessage& msg){
                    if (msg.severity() == QOpenGLDebugMessage::NotificationSeverity)
                        return;
                    if (msg.id() == 131185 || msg.id() == 131169)
                        return;
                    const QString line = QString("GL: [%1] %2 (id=%3)")
                                             .arg(msg.severity())
                                             .arg(msg.message())
                                             .arg(msg.id());
                    if (msg.id() == 1281)
                    {
                        LogSink::instance().log(QString("%1 | phase=%2")
                                                    .arg(line)
                                                    .arg(glPhase_.isEmpty() ? "unknown" : glPhase_));
                    }
                    else
                    {
                        LogSink::instance().log(line);
                    }
                });
        glLogger_.startLogging(QOpenGLDebugLogger::SynchronousLogging);
        glLogger_.enableMessages();
        LogSink::instance().log("GL debug logger initialized.");
    }

    // Many War3 assets have inconsistent winding. For a preview tool, disable culling
    // so models show up reliably.
    glDisable(GL_CULL_FACE);

    isGles_ = QOpenGLContext::currentContext()
                            ? QOpenGLContext::currentContext()->isOpenGLES()
                            : false;
    const QString glslHeader = isGles_ ? "#version 300 es\n" : "#version 330 core\n";
    const QString glslFragPreamble = isGles_ ? "precision mediump float;\n" : "";

    // Mesh shader: textured + lambert, with optional alpha test.
    program_.addShaderFromSourceCode(QOpenGLShader::Vertex, QString(R"GLSL(
        %1
        layout(location=0) in vec3 aPos;
        layout(location=1) in vec3 aNrm;
        layout(location=2) in vec2 aUV;

        uniform mat4 uMVP;
        uniform mat3 uNormalMat;

        out vec3 vNrm;
        out vec2 vUV;

        void main(){
            gl_Position = uMVP * vec4(aPos, 1.0);
            vNrm = normalize(uNormalMat * aNrm);
            vUV = aUV;
        }
    )GLSL").arg(glslHeader));

    program_.addShaderFromSourceCode(QOpenGLShader::Fragment, QString(R"GLSL(
        %1
        %2
        in vec3 vNrm;
        in vec2 vUV;

        uniform sampler2D uTex;
        uniform int uHasTex;
        uniform int uAlphaTest;
        uniform float uAlphaCutoff;
        uniform float uMatAlpha;
        uniform int uUnshaded;

        out vec4 FragColor;

        void main(){
            vec4 base = vec4(0.78, 0.78, 0.78, 1.0);
            if(uHasTex != 0){
                base = texture(uTex, vUV);
            }
            base.a *= uMatAlpha;

            if(uAlphaTest != 0 && base.a < uAlphaCutoff){
                discard;
            }

            float lit = 1.0;
            if(uUnshaded == 0){
                vec3 n = normalize(vNrm);
                vec3 l = normalize(vec3(0.3, 0.5, 0.8));
                lit = max(dot(n, l), 0.15);
            }

            FragColor = vec4(base.rgb * lit, base.a);
        }
    )GLSL").arg(glslHeader, glslFragPreamble));

    programReady_ = program_.link();
    if (!programReady_)
    {
        emit statusTextChanged("Mesh shader link failed: " + program_.log());
        LogSink::instance().log("Mesh shader link failed: " + program_.log());
    }
    else if (!program_.log().isEmpty())
    {
        LogSink::instance().log("Mesh shader log: " + program_.log());
    }

    // Particle shader (unlit)
    particleProgram_.addShaderFromSourceCode(QOpenGLShader::Vertex, QString(R"GLSL(
        %1
        layout(location=0) in vec3 aPos;
        layout(location=1) in vec2 aUV;
        layout(location=2) in vec4 aColor;

        uniform mat4 uMVP;

        out vec2 vUV;
        out vec4 vColor;

        void main(){
            gl_Position = uMVP * vec4(aPos, 1.0);
            vUV = aUV;
            vColor = aColor;
        }
    )GLSL").arg(glslHeader));

    particleProgram_.addShaderFromSourceCode(QOpenGLShader::Fragment, QString(R"GLSL(
        %1
        %2
        in vec2 vUV;
        in vec4 vColor;

        uniform sampler2D uTex;
        uniform int uAlphaTest;
        uniform float uAlphaCutoff;

        out vec4 FragColor;

        void main(){
            vec4 t = texture(uTex, vUV) * vColor;
            if(uAlphaTest != 0 && t.a < uAlphaCutoff)
                discard;
            FragColor = t;
        }
    )GLSL").arg(glslHeader, glslFragPreamble));

    particleProgramReady_ = particleProgram_.link();
    if (!particleProgramReady_)
    {
        emit statusTextChanged("Particle shader link failed: " + particleProgram_.log());
        LogSink::instance().log("Particle shader link failed: " + particleProgram_.log());
    }
    else if (!particleProgram_.log().isEmpty())
    {
        LogSink::instance().log("Particle shader log: " + particleProgram_.log());
    }

    debugProgram_.addShaderFromSourceCode(QOpenGLShader::Vertex, QString(R"GLSL(
        %1
        layout(location=0) in vec3 aPos;
        layout(location=1) in vec4 aColor;

        uniform mat4 uMVP;

        out vec4 vColor;

        void main(){
            gl_Position = uMVP * vec4(aPos, 1.0);
            vColor = aColor;
        }
    )GLSL").arg(glslHeader));

    debugProgram_.addShaderFromSourceCode(QOpenGLShader::Fragment, QString(R"GLSL(
        %1
        %2
        in vec4 vColor;
        out vec4 FragColor;
        void main(){
            FragColor = vColor;
        }
    )GLSL").arg(glslHeader, glslFragPreamble));

    debugProgramReady_ = debugProgram_.link();
    if (!debugProgramReady_)
        LogSink::instance().log("Debug shader link failed: " + debugProgram_.log());
    else if (!debugProgram_.log().isEmpty())
        LogSink::instance().log("Debug shader log: " + debugProgram_.log());

    // Particle buffer
    glGenVertexArrays(1, &pVao_);
    glBindVertexArray(pVao_);
    glGenBuffers(1, &pVbo_);
    glBindBuffer(GL_ARRAY_BUFFER, pVbo_);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(ParticleVertex), (void*)offsetof(ParticleVertex, px));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(ParticleVertex), (void*)offsetof(ParticleVertex, u));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(ParticleVertex), (void*)offsetof(ParticleVertex, r));

    glBindVertexArray(0);

    // Debug buffer
    glGenVertexArrays(1, &debugVao_);
    glBindVertexArray(debugVao_);
    glGenBuffers(1, &debugVbo_);
    glBindBuffer(GL_ARRAY_BUFFER, debugVbo_);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(DebugVertex), (void*)offsetof(DebugVertex, px));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(DebugVertex), (void*)offsetof(DebugVertex, r));

    glBindVertexArray(0);

    // Sanity triangle (hardcoded)
    {
        const DebugVertex tri[3] = {
            {0.0f, 0.0f, 0.0f, 0.95f, 0.2f, 0.2f, 1.0f},
            {0.35f, 0.0f, 0.0f, 0.2f, 0.95f, 0.2f, 1.0f},
            {0.0f, 0.35f, 0.0f, 0.2f, 0.2f, 0.95f, 1.0f},
        };
        glGenVertexArrays(1, &sanityVao_);
        glBindVertexArray(sanityVao_);
        glGenBuffers(1, &sanityVbo_);
        glBindBuffer(GL_ARRAY_BUFFER, sanityVbo_);
        glBufferData(GL_ARRAY_BUFFER, sizeof(tri), tri, GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(DebugVertex), (void*)offsetof(DebugVertex, px));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(DebugVertex), (void*)offsetof(DebugVertex, r));
        glBindVertexArray(0);
    }

    rebuildGpuBuffers();
}

void GLModelView::resizeGL(int w, int h)
{
    updateProjection(w, h);
}

void GLModelView::tickAnimation()
{
    // Only advance if we have a model loaded.
    if (!model_)
        return;

    // dt in seconds
    const qint64 ns = frameTimer_.nsecsElapsed();
    frameTimer_.restart();
    float dt = float(ns) / 1.0e9f;
    dt = clampf(dt, 0.0f, 0.1f);

    updateEmitters(dt);
    update(); // trigger repaint
}

void GLModelView::updateEmitters(float dtSeconds)
{
    if (!model_)
        return;

    // Advance local time (monotonic)
    localTimeMs_ += std::uint32_t(dtSeconds * 1000.0f * playbackSpeed_);

    // Determine global time (sequence mapping)
    std::uint32_t globalTimeMs = localTimeMs_;
    if (!model_->sequences.empty())
    {
        const auto& seq = model_->sequences[std::size_t(std::max(0, currentSeq_))];
        const std::uint32_t start = seq.startMs;
        const std::uint32_t end = std::max(seq.endMs, seq.startMs + 1);
        const std::uint32_t len = end - start;
        const std::uint32_t local = (len != 0) ? (localTimeMs_ % len) : 0;
        globalTimeMs = start + local;
    }
    lastGlobalTimeMs_ = globalTimeMs;

    // RNG (fixed seed per run; adequate for preview)
    static thread_local std::mt19937 rng(1337u);
    std::uniform_real_distribution<float> u01(0.0f, 1.0f);
    auto randSigned = [&]() { return u01(rng) * 2.0f - 1.0f; };

    // Ensure runtime storage matches emitter count
    if (runtimeEmitters2_.size() != model_->emitters2.size())
        runtimeEmitters2_.assign(model_->emitters2.size(), {});

    for (std::size_t ei = 0; ei < model_->emitters2.size(); ++ei)
    {
        const auto& e = model_->emitters2[ei];
        auto& rt = runtimeEmitters2_[ei];

        const float vis = clampf(sampleTrackFloat(e.trackVisibility, globalTimeMs, 1.0f, *model_), 0.0f, 1.0f);
        if (vis <= 0.001f)
        {
            // Still age existing particles so they fade out naturally.
        }

        const float speed = sampleTrackFloat(e.trackSpeed, globalTimeMs, e.speed, *model_);
        const float variation = sampleTrackFloat(e.trackVariation, globalTimeMs, e.variation, *model_);
        const float latitude = sampleTrackFloat(e.trackLatitude, globalTimeMs, e.latitude, *model_);
        const float emissionRate = std::max(0.0f, sampleTrackFloat(e.trackEmissionRate, globalTimeMs, e.emissionRate, *model_));
        const float gravity = sampleTrackFloat(e.trackGravity, globalTimeMs, e.gravity, *model_);
        const float lifespan = std::max(0.01f, sampleTrackFloat(e.trackLifespan, globalTimeMs, e.lifespan, *model_));
        const float width = sampleTrackFloat(e.trackWidth, globalTimeMs, e.width, *model_);
        const float length = sampleTrackFloat(e.trackLength, globalTimeMs, e.length, *model_);

        // Spawn particles
        if (vis > 0.001f && emissionRate > 0.0f)
        {
            rt.spawnAccum += double(emissionRate) * double(dtSeconds);
            int toSpawn = int(rt.spawnAccum);
            if (toSpawn > 0)
            {
                rt.spawnAccum -= double(toSpawn);
                toSpawn = std::min(toSpawn, 200); // safety cap

                QVector3D pivot(0,0,0);
                if (e.objectId >= 0 && std::size_t(e.objectId) < model_->pivots.size())
                {
                    const auto& p = model_->pivots[std::size_t(e.objectId)];
                    pivot = QVector3D(p.x, p.y, p.z);
                }

                for (int i = 0; i < toSpawn; ++i)
                {
                    Particle p;
                    p.age = 0.0f;
                    p.life = lifespan;

                    // Initial position: pivot + small scatter in X/Y using width/length
                    const float sx = randSigned() * width;
                    const float sy = randSigned() * length;
                    p.pos = pivot + QVector3D(sx, sy, 0.0f);

                    // Direction within latitude (simplified)
                    const float lat = latitude;
                    const float ax = randSigned() * lat;
                    const float ay = randSigned() * lat;

                    QVector3D dir(0,0,1);
                    QQuaternion qx = QQuaternion::fromAxisAndAngle(1,0,0, ax * 57.2957795f);
                    QQuaternion qy = QQuaternion::fromAxisAndAngle(0,1,0, ay * 57.2957795f);
                    dir = (qy * qx).rotatedVector(dir);
                    dir.normalize();

                    const float sp = speed + randSigned() * variation;
                    p.vel = dir * sp;

                    rt.particles.push_back(p);
                }
            }
        }

        // Update existing particles
        for (auto& p : rt.particles)
        {
            p.age += dtSeconds;
            // gravity pulls down in Z
            p.vel.setZ(p.vel.z() - gravity * dtSeconds);
            p.pos += p.vel * dtSeconds;
        }

        // Remove dead
        rt.particles.erase(
            std::remove_if(rt.particles.begin(), rt.particles.end(),
                           [](const Particle& p) { return p.age >= p.life; }),
            rt.particles.end()
        );

        // Keep count bounded for safety
        if (rt.particles.size() > 5000)
            rt.particles.erase(rt.particles.begin(), rt.particles.end() - 5000);
    }
}

void GLModelView::buildDebugGeometry()
{
    debugVerts_.clear();

    const float axisLen = std::max(1.0f, boundsRadius_ * 0.75f);
    const QVector3D o = modelCenter_;

    auto pushLine = [&](const QVector3D& a, const QVector3D& b, const QVector4D& c)
    {
        DebugVertex v0{a.x(), a.y(), a.z(), c.x(), c.y(), c.z(), c.w()};
        DebugVertex v1{b.x(), b.y(), b.z(), c.x(), c.y(), c.z(), c.w()};
        debugVerts_.push_back(v0);
        debugVerts_.push_back(v1);
    };

    // Axes
    pushLine(o, o + QVector3D(axisLen, 0, 0), QVector4D(1, 0, 0, 1));
    pushLine(o, o + QVector3D(0, axisLen, 0), QVector4D(0, 1, 0, 1));
    pushLine(o, o + QVector3D(0, 0, axisLen), QVector4D(0, 0, 1, 1));

    // AABB edges
    const QVector3D mn = boundsMin_;
    const QVector3D mx = boundsMax_;
    const QVector3D c(0.9f, 0.9f, 0.2f);
    const QVector4D col(c.x(), c.y(), c.z(), 1.0f);

    QVector3D v[8] = {
        {mn.x(), mn.y(), mn.z()},
        {mx.x(), mn.y(), mn.z()},
        {mx.x(), mx.y(), mn.z()},
        {mn.x(), mx.y(), mn.z()},
        {mn.x(), mn.y(), mx.z()},
        {mx.x(), mn.y(), mx.z()},
        {mx.x(), mx.y(), mx.z()},
        {mn.x(), mx.y(), mx.z()}
    };

    const int e[12][2] = {
        {0,1},{1,2},{2,3},{3,0},
        {4,5},{5,6},{6,7},{7,4},
        {0,4},{1,5},{2,6},{3,7}
    };
    for (const auto& edge : e)
        pushLine(v[edge[0]], v[edge[1]], col);
}

void GLModelView::drawDebug(const QMatrix4x4& mvp)
{
    if (!debugProgramReady_ || debugVao_ == 0 || debugVerts_.empty())
        return;

    setGlPhase("debug-lines");
    debugProgram_.bind();
    debugProgram_.setUniformValue("uMVP", mvp);

    glBindVertexArray(debugVao_);
    glBindBuffer(GL_ARRAY_BUFFER, debugVbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 GLsizeiptr(debugVerts_.size() * sizeof(DebugVertex)),
                 debugVerts_.data(),
                 GL_DYNAMIC_DRAW);

    glDisable(GL_DEPTH_TEST);
    glLineWidth(1.0f);
    glDrawArrays(GL_LINES, 0, GLsizei(debugVerts_.size()));
    lastDrawCalls_ += 1;
    glLineWidth(1.0f);
    glEnable(GL_DEPTH_TEST);

    glBindVertexArray(0);
    debugProgram_.release();
}

void GLModelView::updateSkinning(std::uint32_t globalTimeMs)
{
    if (!model_)
        return;
    if (model_->bindVertices.empty() || model_->vertexGroups.size() != model_->bindVertices.size())
        return;
    if (model_->skinGroups.empty() || model_->nodes.empty())
        return;
    if (vbo_ == 0)
        return;

    if (skinnedVertices_.size() != model_->bindVertices.size())
        skinnedVertices_ = model_->bindVertices;

    const std::size_t nodeCount = model_->nodes.size();
    std::vector<QMatrix4x4> nodeWorld(nodeCount);
    std::vector<int> state(nodeCount, 0);

    auto adjustParentForChild = [&](const QMatrix4x4& parent, std::uint32_t flags) -> QMatrix4x4
    {
        const bool inheritT = (flags & NODE_DONT_INHERIT_TRANSLATION) == 0;
        const bool inheritR = (flags & NODE_DONT_INHERIT_ROTATION) == 0;
        const bool inheritS = (flags & NODE_DONT_INHERIT_SCALING) == 0;

        QVector3D t(parent(0,3), parent(1,3), parent(2,3));
        QVector3D c0(parent(0,0), parent(1,0), parent(2,0));
        QVector3D c1(parent(0,1), parent(1,1), parent(2,1));
        QVector3D c2(parent(0,2), parent(1,2), parent(2,2));

        float sx = c0.length();
        float sy = c1.length();
        float sz = c2.length();
        if (sx > 0.000001f) c0 /= sx;
        if (sy > 0.000001f) c1 /= sy;
        if (sz > 0.000001f) c2 /= sz;

        if (!inheritR)
        {
            c0 = QVector3D(1,0,0);
            c1 = QVector3D(0,1,0);
            c2 = QVector3D(0,0,1);
        }
        if (!inheritS)
        {
            sx = 1.0f;
            sy = 1.0f;
            sz = 1.0f;
        }
        if (!inheritT)
        {
            t = QVector3D(0,0,0);
        }

        QMatrix4x4 out;
        out.setToIdentity();
        out.setColumn(0, QVector4D(c0.x()*sx, c0.y()*sx, c0.z()*sx, 0.0f));
        out.setColumn(1, QVector4D(c1.x()*sy, c1.y()*sy, c1.z()*sy, 0.0f));
        out.setColumn(2, QVector4D(c2.x()*sz, c2.y()*sz, c2.z()*sz, 0.0f));
        out.setColumn(3, QVector4D(t.x(), t.y(), t.z(), 1.0f));
        return out;
    };

    auto buildNode = [&](auto&& self, std::size_t idx) -> void
    {
        if (idx >= nodeCount)
            return;
        if (state[idx] == 2)
            return;
        if (state[idx] == 1)
        {
            nodeWorld[idx].setToIdentity();
            state[idx] = 2;
            return;
        }
        state[idx] = 1;

        const auto& n = model_->nodes[idx];
        const Vec3 defT{0,0,0};
        const Vec3 defS{1,1,1};
        const Vec4 defR{0,0,0,1};

        const Vec3 t = sampleTrackVec3(n.trackTranslation, globalTimeMs, defT, *model_);
        const Vec3 s = sampleTrackVec3(n.trackScaling, globalTimeMs, defS, *model_);
        Vec4 r = sampleTrackQuat(n.trackRotation, globalTimeMs, defR, *model_);

        const QVector3D pivot(n.pivot.x, n.pivot.y, n.pivot.z);
        QQuaternion q(r.w, r.x, r.y, r.z);

        QMatrix4x4 local;
        local.setToIdentity();
        local.translate(pivot.x() + t.x, pivot.y() + t.y, pivot.z() + t.z);
        local.rotate(q);
        local.scale(s.x, s.y, s.z);
        local.translate(-pivot);

        if (n.parentId >= 0 && std::size_t(n.parentId) < nodeCount)
        {
            self(self, std::size_t(n.parentId));
            const QMatrix4x4 parentAdj = adjustParentForChild(
                nodeWorld[std::size_t(n.parentId)],
                n.flags);
            nodeWorld[idx] = parentAdj * local;
        }
        else
        {
            nodeWorld[idx] = local;
        }

        state[idx] = 2;
    };

    for (std::size_t i = 0; i < nodeCount; ++i)
        buildNode(buildNode, i);

    // Warcraft 3 classic MDX (v800) uses "matrix groups" (a jagged list of node indices) without
    // explicit per-vertex weights. The common approach (and what the WC3 pipeline effectively
    // supports) is to take up to 4 matrices for the vertex group, transform the vertex by each,
    // sum the results, and divide by the number of matrices (a simple average). 
    // See e.g. the mdx-m3-viewer shader and related discussions.
    //
    // This is *not* mathematically correct skinning, but it matches real-world WC3 assets better
    // than picking a single bone for multi-matrix groups.

    auto skinAverage = [&](const ModelVertex& base, const ModelData::SkinGroup& group, ModelVertex& outV) -> bool
    {
        // WC3 SD skinning uses matrix groups without explicit weights.
        // Most geosets need up to 4 bones per vertex, but a few use "extended vertex groups"
        // and require up to 8 bones (mdx-m3-viewer chooses this when any MTGC entry > 4).
        // We support up to 8 here, and average the transformed results.
        const int maxBones = (group.nodeIndices.size() > 4) ? 8 : 4;

        int bones[8] = {-1,-1,-1,-1,-1,-1,-1,-1};
        int boneCount = 0;
        for (int idx : group.nodeIndices)
        {
            if (idx < 0 || std::size_t(idx) >= nodeWorld.size())
                continue;

            // Avoid duplicates (bad models sometimes repeat indices).
            bool dup = false;
            for (int j = 0; j < boneCount; ++j)
            {
                if (bones[j] == idx) { dup = true; break; }
            }
            if (dup)
                continue;

            bones[boneCount++] = idx;
            if (boneCount == maxBones)
                break;
        }

        if (boneCount <= 0)
            return false;

        const QVector4D p4(base.px, base.py, base.pz, 1.0f);
        const QVector4D n4(base.nx, base.ny, base.nz, 0.0f);

        QVector4D sumP(0,0,0,0);
        QVector4D sumN(0,0,0,0);
        for (int i = 0; i < boneCount; ++i)
        {
            const QMatrix4x4& m = nodeWorld[std::size_t(bones[i])];
            sumP += m * p4;
            sumN += m * n4;
        }

        const float inv = 1.0f / float(boneCount);
        const QVector4D avgP = sumP * inv;

        QVector3D nn(sumN.x(), sumN.y(), sumN.z());
        if (nn.lengthSquared() > 0.000001f)
            nn.normalize();
        else
            nn = QVector3D(0,0,1);

        outV = base;
        outV.px = avgP.x();
        outV.py = avgP.y();
        outV.pz = avgP.z();
        outV.nx = nn.x();
        outV.ny = nn.y();
        outV.nz = nn.z();
        return true;
    };

    for (std::size_t i = 0; i < model_->bindVertices.size(); ++i)
    {
        const auto& base = model_->bindVertices[i];
        if (i >= model_->vertexGroups.size())
        {
            skinnedVertices_[i] = base;
            continue;
        }
        const std::uint16_t gid = model_->vertexGroups[i];
        if (gid >= model_->skinGroups.size())
        {
            skinnedVertices_[i] = base;
            continue;
        }
        const auto& group = model_->skinGroups[gid];
        if (group.nodeIndices.empty())
        {
            skinnedVertices_[i] = base;
            continue;
        }

        ModelVertex v;
        if (skinAverage(base, group, v))
        {
            skinnedVertices_[i] = v;
        }
        else
        {
            skinnedVertices_[i] = base;
        }
    }

    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferSubData(GL_ARRAY_BUFFER,
                    0,
                    GLsizeiptr(skinnedVertices_.size() * sizeof(ModelVertex)),
                    skinnedVertices_.data());
}

void GLModelView::updateStatusText()
{
    if (statusTimer_.isValid() && statusTimer_.elapsed() < 250)
        return;
    if (!statusTimer_.isValid())
        statusTimer_.start();
    else
        statusTimer_.restart();

    const std::size_t verts = model_ ? model_->vertices.size() : 0;
    const std::size_t tris = model_ ? (model_->indices.size() / 3) : 0;
    const std::size_t geosets = model_ ? (model_->geosetCount != 0 ? model_->geosetCount : model_->subMeshes.size()) : 0;
    const std::size_t materials = model_ ? model_->materials.size() : 0;
    const std::size_t textures = model_ ? model_->textures.size() : 0;

    QString extra;
    if (model_ && verts == 0)
        extra = model_->emitters2.empty() ? " | empty mesh" : " | particle-only";

    emit statusTextChanged(QString("%1 | v:%2 t:%3 g:%4 m:%5 tex:%6 dc:%7 fps:%8%9")
                               .arg(displayName_)
                               .arg(verts)
                               .arg(tris)
                               .arg(geosets)
                               .arg(materials)
                               .arg(textures)
                               .arg(lastDrawCalls_)
                               .arg(QString::number(fps_, 'f', 1))
                               .arg(extra));
}

void GLModelView::paintGL()
{
    const float dpr = devicePixelRatioF();
    const int fbw = int(width() * dpr);
    const int fbh = int(height() * dpr);
    glViewport(0, 0, fbw, fbh);
    setGlPhase("clear");
    glClearColor(0.0f, 0.0f, 0.0f, backgroundAlpha_);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    lastDrawCalls_ = 0;
    if (!model_)
        return;

    // Camera: orbit around model center
    QMatrix4x4 view;
    view.setToIdentity();
    view.translate(0, 0, -distance_);
    view.rotate(pitch_, 1, 0, 0);
    view.rotate(yaw_, 0, 1, 0);
    view.rotate(roll_, 0, 0, 1);
    view.translate(-(modelCenter_ + panOffset_));

    QMatrix4x4 modelM;
    modelM.setToIdentity();

    const QMatrix4x4 mvp = proj_ * view * modelM;
    const QMatrix3x3 normalMat = modelM.normalMatrix();

    updateSkinning(lastGlobalTimeMs_);

    // --- Draw mesh (if any)
    if (programReady_ && vao_ != 0 && !model_->indices.empty())
    {
        setGlPhase("mesh");
        if (wireframe_ && !isGles_)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        program_.bind();
        program_.setUniformValue("uMVP", mvp);
        program_.setUniformValue("uNormalMat", normalMat);

        glBindVertexArray(vao_);

        glDisable(GL_CULL_FACE);

        // Pass 1: opaque + alpha-tested (depth write on)
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);

        auto drawSubmesh = [&](const GpuSubmesh& sm, bool transparentPass)
        {
            const std::uint32_t matId = sm.materialId;
            if (matId >= model_->materials.size())
                return;

            const auto& mat = model_->materials[matId];
            const auto& layer = mat.layer;

            const bool unshaded = (layer.shadingFlags & LAYER_UNSHADED) != 0;
            const bool noDepthTest = (layer.shadingFlags & LAYER_NODEPTH) != 0;
            const bool noDepthSet  = (layer.shadingFlags & LAYER_NODEPTHSET) != 0;

            const std::uint32_t filter = layer.filterMode;

            const bool alphaTest = alphaTestEnabled_ && (filter == 1);
            const bool blended   = (filter == 2 || filter == 3 || filter == 4 || filter == 5 || filter == 6);

            if (transparentPass != blended)
                return;

            if (noDepthTest) glDisable(GL_DEPTH_TEST); else glEnable(GL_DEPTH_TEST);
            glDepthMask(noDepthSet ? GL_FALSE : GL_TRUE);

            if (blended)
            {
                glEnable(GL_BLEND);
                switch (filter)
                {
                case 3: // Additive
                case 4: // Add alpha (approx)
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
                    break;
                case 5: // Modulate
                    glBlendFunc(GL_DST_COLOR, GL_ZERO);
                    break;
                case 6: // Modulate2x (approx)
                    glBlendFunc(GL_DST_COLOR, GL_SRC_COLOR);
                    break;
                default: // Blend
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                    break;
                }
            }
            else
            {
                glDisable(GL_BLEND);
            }

            const GLuint tex = getOrCreateTexture(layer.textureId);
            const bool hasTex = (tex != placeholderTex_ && tex != 0);

            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, tex);

            program_.setUniformValue("uTex", 0);
            program_.setUniformValue("uHasTex", hasTex ? 1 : 0);
            program_.setUniformValue("uAlphaTest", alphaTest ? 1 : 0);
            program_.setUniformValue("uAlphaCutoff", 0.5f);
            program_.setUniformValue("uMatAlpha", clampf(layer.alpha, 0.0f, 1.0f));
            program_.setUniformValue("uUnshaded", unshaded ? 1 : 0);

            glDrawElements(
                GL_TRIANGLES,
                GLsizei(sm.indexCount),
                GL_UNSIGNED_INT,
                (void*)(uintptr_t(sm.indexOffset * sizeof(std::uint32_t)))
            );
            lastDrawCalls_ += 1;
        };

        for (const auto& sm : gpuSubmeshes_)
            drawSubmesh(sm, false);

        // Pass 2: blended materials
        std::vector<std::size_t> order;
        order.reserve(gpuSubmeshes_.size());
        for (std::size_t i = 0; i < gpuSubmeshes_.size(); ++i) order.push_back(i);
        std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b){
            const auto& ma = model_->materials[gpuSubmeshes_[a].materialId];
            const auto& mb = model_->materials[gpuSubmeshes_[b].materialId];
            return ma.priorityPlane < mb.priorityPlane;
        });

        for (auto idx : order)
            drawSubmesh(gpuSubmeshes_[idx], true);

        glBindVertexArray(0);
        program_.release();

        if (wireframe_ && !isGles_)
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // Reset to a known baseline before other passes.
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDisable(GL_CULL_FACE);
    }

    // --- Draw particles (PRE2)
    if (particleProgramReady_ && pVao_ != 0 && !model_->emitters2.empty())
    {
        setGlPhase("particles");
        particleProgram_.bind();
        particleProgram_.setUniformValue("uMVP", mvp);
        particleProgram_.setUniformValue("uTex", 0);

        // We generally don't want particles writing depth.
        glEnable(GL_BLEND);
        glDepthMask(GL_FALSE);
        glEnable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);

        // Camera basis for billboards
        const QMatrix4x4 invView = view.inverted();
        const QVector3D camRight = invView.column(0).toVector3D().normalized();
        const QVector3D camUp    = invView.column(1).toVector3D().normalized();
        const QVector3D camFwd   = (-invView.column(2).toVector3D()).normalized();

        glBindVertexArray(pVao_);

        for (std::size_t ei = 0; ei < model_->emitters2.size(); ++ei)
        {
            const auto& e = model_->emitters2[ei];
            const auto& rt = (ei < runtimeEmitters2_.size()) ? runtimeEmitters2_[ei] : RuntimeEmitter2{};

            if (rt.particles.empty())
                continue;

            // Resolve texture
            const GLuint tex = (e.textureId >= 0) ? getOrCreateTexture(std::uint32_t(e.textureId)) : placeholderTex_;
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, tex);

            // Blend mode (match WC3 as closely as reasonable)
            const std::uint32_t f = e.filterMode;
            const bool alphaKey = (f == 4); // AlphaKey
            particleProgram_.setUniformValue("uAlphaTest", 0);
            particleProgram_.setUniformValue("uAlphaCutoff", 0.0f);
            if (alphaKey)
            {
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            }
            else
            {
                switch (f)
                {
                case 1: // Additive
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
                    break;
                case 2: // Modulate
                    glBlendFunc(GL_ZERO, GL_SRC_COLOR);
                    break;
                case 3: // Modulate2x (approx)
                    glBlendFunc(GL_DST_COLOR, GL_SRC_COLOR);
                    break;
                default: // Blend
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                    break;
                }
            }

            // Build CPU vertices for this emitter (two triangles per particle)
            particleVerts_.clear();
            particleVerts_.reserve(rt.particles.size() * 6);

            const int totalFrames = int(std::max<std::uint32_t>(1, e.rows * e.columns));
            const float invCols = 1.0f / float(std::max<std::uint32_t>(1, e.columns));
            const float invRows = 1.0f / float(std::max<std::uint32_t>(1, e.rows));

            auto evalSegment = [&](float lifeT, Vec3& outColor, float& outAlpha, float& outScale)
            {
                const float mid = clampf(e.timeMiddle, 0.01f, 0.99f);

                auto toAlpha = [&](std::uint8_t a){ return float(a) / 255.0f; };

                if (lifeT <= mid)
                {
                    const float t = lifeT / mid;
                    outColor.x = lerpf(e.segmentColor[0].x, e.segmentColor[1].x, t);
                    outColor.y = lerpf(e.segmentColor[0].y, e.segmentColor[1].y, t);
                    outColor.z = lerpf(e.segmentColor[0].z, e.segmentColor[1].z, t);
                    outAlpha = lerpf(toAlpha(e.segmentAlpha[0]), toAlpha(e.segmentAlpha[1]), t);
                    outScale = lerpf(e.segmentScaling[0], e.segmentScaling[1], t);
                }
                else
                {
                    const float t = (lifeT - mid) / (1.0f - mid);
                    outColor.x = lerpf(e.segmentColor[1].x, e.segmentColor[2].x, t);
                    outColor.y = lerpf(e.segmentColor[1].y, e.segmentColor[2].y, t);
                    outColor.z = lerpf(e.segmentColor[1].z, e.segmentColor[2].z, t);
                    outAlpha = lerpf(toAlpha(e.segmentAlpha[1]), toAlpha(e.segmentAlpha[2]), t);
                    outScale = lerpf(e.segmentScaling[1], e.segmentScaling[2], t);
                }

                // Scaling in MDX is in percent.
                outScale = outScale / 100.0f;
            };

            for (const auto& p : rt.particles)
            {
                const float tLife = clampf(p.age / std::max(0.001f, p.life), 0.0f, 1.0f);

                Vec3 col;
                float alpha;
                float scale;
                evalSegment(tLife, col, alpha, scale);

                // Sprite frame selection (simple; ignores head/tail intervals for now)
                int frame = int(tLife * float(totalFrames));
                if (frame >= totalFrames) frame = totalFrames - 1;
                if (frame < 0) frame = 0;

                const int fr = frame / int(std::max<std::uint32_t>(1, e.columns));
                const int fc = frame % int(std::max<std::uint32_t>(1, e.columns));

                const float u0 = float(fc) * invCols;
                const float v0 = float(fr) * invRows;
                const float u1 = u0 + invCols;
                const float v1 = v0 + invRows;

                const QVector3D pos = p.pos;

                const float half = 0.5f * std::max(0.01f, scale);
                const float r = col.x;
                const float g = col.y;
                const float b = col.z;
                const float a = alpha;

                auto pushTri = [&](const QVector3D& p0, float tu0, float tv0,
                                   const QVector3D& p1, float tu1, float tv1,
                                   const QVector3D& p2, float tu2, float tv2)
                {
                    ParticleVertex v{};
                    v.r = r; v.g = g; v.b = b; v.a = a;

                    v.px = p0.x(); v.py = p0.y(); v.pz = p0.z(); v.u = tu0; v.v = tv0;
                    particleVerts_.push_back(v);
                    v.px = p1.x(); v.py = p1.y(); v.pz = p1.z(); v.u = tu1; v.v = tv1;
                    particleVerts_.push_back(v);
                    v.px = p2.x(); v.py = p2.y(); v.pz = p2.z(); v.u = tu2; v.v = tv2;
                    particleVerts_.push_back(v);
                };

                if (e.headOrTail != 0 && e.tailLength > 0.0001f)
                {
                    QVector3D dir = p.vel;
                    if (dir.lengthSquared() < 1e-6f)
                        dir = camFwd;
                    dir.normalize();

                    QVector3D side = QVector3D::crossProduct(camFwd, dir);
                    if (side.lengthSquared() < 1e-6f)
                        side = camRight;
                    side.normalize();

                    const QVector3D p0 = pos;
                    const QVector3D p1 = pos - dir * e.tailLength;

                    const QVector3D a0 = p0 + side * half;
                    const QVector3D a1 = p0 - side * half;
                    const QVector3D b0 = p1 + side * half;
                    const QVector3D b1 = p1 - side * half;

                    // two triangles (a0,a1,b1) and (a0,b1,b0)
                    pushTri(a0, u0, v0, a1, u1, v0, b1, u1, v1);
                    pushTri(a0, u0, v0, b1, u1, v1, b0, u0, v1);
                }
                else
                {
                    const QVector3D right = camRight * half;
                    const QVector3D up = camUp * half;

                    const QVector3D p00 = pos - right - up;
                    const QVector3D p10 = pos + right - up;
                    const QVector3D p11 = pos + right + up;
                    const QVector3D p01 = pos - right + up;

                    // two triangles
                    pushTri(p00, u0, v0, p10, u1, v0, p11, u1, v1);
                    pushTri(p00, u0, v0, p11, u1, v1, p01, u0, v1);
                }
            }

            if (particleVerts_.empty())
                continue;

            glBindBuffer(GL_ARRAY_BUFFER, pVbo_);
            glBufferData(GL_ARRAY_BUFFER,
                         GLsizeiptr(particleVerts_.size() * sizeof(ParticleVertex)),
                         particleVerts_.data(),
                         GL_DYNAMIC_DRAW);

            glDrawArrays(GL_TRIANGLES, 0, GLsizei(particleVerts_.size()));
            lastDrawCalls_ += 1;
        }

        glBindVertexArray(0);

        // Restore defaults
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        glDisable(GL_CULL_FACE);
        particleProgram_.release();
    }

    if (!fpsTimer_.isValid())
        fpsTimer_.start();
    fpsFrames_ += 1;
    const qint64 fpsElapsed = fpsTimer_.elapsed();
    if (fpsElapsed >= 1000)
    {
        fps_ = float(fpsFrames_) * 1000.0f / float(fpsElapsed);
        fpsFrames_ = 0;
        fpsTimer_.restart();
    }

    if (model_ && !model_->vertices.empty() && lastDrawCalls_ == 0 && !loggedBlank_)
    {
        LogSink::instance().log(QString("Blank draw: target=%1,%2,%3 dist=%4 near=%5 far=%6 drawCalls=%7 alphaTest=%8 cull=%9 blend=%10")
                                    .arg(modelCenter_.x()).arg(modelCenter_.y()).arg(modelCenter_.z())
                                    .arg(distance_)
                                    .arg(near_)
                                    .arg(far_)
                                    .arg(lastDrawCalls_)
                                    .arg(alphaTestEnabled_ ? "on" : "off")
                                    .arg("off")
                                    .arg("per-material"));
        loggedBlank_ = true;
    }

    updateStatusText();
}

void GLModelView::mousePressEvent(QMouseEvent* e)
{
    lastMouse_ = e->pos();
}

void GLModelView::mouseMoveEvent(QMouseEvent* e)
{
    const QPoint delta = e->pos() - lastMouse_;
    lastMouse_ = e->pos();

    const bool panMode = (e->buttons() & Qt::RightButton) ||
                         ((e->buttons() & Qt::LeftButton) && (e->modifiers() & Qt::ControlModifier));
    const bool rollMode = (e->buttons() & Qt::LeftButton) && (e->modifiers() & Qt::ShiftModifier);

    if (panMode)
    {
        QMatrix4x4 rot;
        rot.setToIdentity();
        rot.rotate(yaw_, 0, 1, 0);
        rot.rotate(pitch_, 1, 0, 0);
        rot.rotate(roll_, 0, 0, 1);
        const QVector3D right = rot.mapVector(QVector3D(1, 0, 0));
        const QVector3D up = rot.mapVector(QVector3D(0, 1, 0));
        const float scale = std::max(0.001f, distance_ * 0.002f);
        panOffset_ += (-right * float(delta.x()) + up * float(delta.y())) * scale;
        emit panChanged(panOffset_.x(), panOffset_.y(), panOffset_.z());
        update();
        return;
    }

    if (rollMode)
    {
        roll_ += delta.x() * 0.4f;
        emit anglesChanged(yaw_, pitch_, roll_);
        update();
        return;
    }

    if (e->buttons() & Qt::LeftButton)
    {
        yaw_ += delta.x() * 0.4f;
        pitch_ += delta.y() * 0.4f;
        pitch_ = clampf(pitch_, -89.0f, 89.0f);
        emit anglesChanged(yaw_, pitch_, roll_);
        update();
    }
}

void GLModelView::wheelEvent(QWheelEvent* e)
{
    const float num = e->angleDelta().y() / 120.0f;
    distance_ *= std::pow(0.90f, num);
    distance_ = clampf(distance_, 0.25f, 10000.0f);
    updateProjection(viewportW_, viewportH_);
    update();
}

void GLModelView::mouseDoubleClickEvent(QMouseEvent* e)
{
    if (e->button() == Qt::LeftButton)
    {
        resetView();
        e->accept();
        return;
    }
    QOpenGLWidget::mouseDoubleClickEvent(e);
}

void GLModelView::keyPressEvent(QKeyEvent* e)
{
    if (e->key() == Qt::Key_F)
    {
        resetView();
        e->accept();
        return;
    }
    if (e->key() == Qt::Key_W)
    {
        wireframe_ = !wireframe_;
        LogSink::instance().log(QString("Wireframe: %1").arg(wireframe_ ? "on" : "off"));
        update();
        e->accept();
        return;
    }
    if (e->key() == Qt::Key_A)
    {
        alphaTestEnabled_ = !alphaTestEnabled_;
        LogSink::instance().log(QString("Alpha test: %1").arg(alphaTestEnabled_ ? "on" : "off"));
        update();
        e->accept();
        return;
    }
    QOpenGLWidget::keyPressEvent(e);
}

void GLModelView::clearGpuResources()
{
    for (auto it = gpuCache_.begin(); it != gpuCache_.end(); ++it)
    {
        if (it->ibo) glDeleteBuffers(1, &it->ibo);
        if (it->vbo) glDeleteBuffers(1, &it->vbo);
        if (it->vao) glDeleteVertexArrays(1, &it->vao);
    }
    gpuCache_.clear();
    ibo_ = 0;
    vbo_ = 0;
    vao_ = 0;

    if (pVbo_) { glDeleteBuffers(1, &pVbo_); pVbo_ = 0; }
    if (pVao_) { glDeleteVertexArrays(1, &pVao_); pVao_ = 0; }
    if (debugVbo_) { glDeleteBuffers(1, &debugVbo_); debugVbo_ = 0; }
    if (debugVao_) { glDeleteVertexArrays(1, &debugVao_); debugVao_ = 0; }
    if (sanityVbo_) { glDeleteBuffers(1, &sanityVbo_); sanityVbo_ = 0; }
    if (sanityVao_) { glDeleteVertexArrays(1, &sanityVao_); sanityVao_ = 0; }

    for (auto& kv : textureCache_)
    {
        if (kv.second.valid && kv.second.id != 0)
            glDeleteTextures(1, &kv.second.id);
    }
    textureCache_.clear();

    if (placeholderTex_ != 0)
    {
        glDeleteTextures(1, &placeholderTex_);
        placeholderTex_ = 0;
    }
    if (teamColorTex_ != 0)
    {
        glDeleteTextures(1, &teamColorTex_);
        teamColorTex_ = 0;
    }
    if (teamGlowTex_ != 0)
    {
        glDeleteTextures(1, &teamGlowTex_);
        teamGlowTex_ = 0;
    }

    gpuSubmeshes_.clear();
}

void GLModelView::rebuildGpuBuffers()
{
    // Mesh buffers are tied to model geometry. Particles have their own buffers created in initializeGL.
    gpuSubmeshes_.clear();

    if (!model_ || model_->vertices.empty() || model_->indices.empty())
    {
        if (placeholderTex_ == 0)
            placeholderTex_ = createPlaceholderTexture();
        return;
    }

    if (!modelPath_.isEmpty())
    {
        auto it = gpuCache_.find(modelPath_);
        if (it != gpuCache_.end())
        {
            vao_ = it->vao;
            vbo_ = it->vbo;
            ibo_ = it->ibo;
            gpuSubmeshes_ = it->submeshes;
            return;
        }
    }

    // Upload vertex/index data
    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);

    glGenBuffers(1, &vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    const bool useSkinning = !model_->skinGroups.empty() &&
                             model_->vertexGroups.size() == model_->vertices.size();
    const auto& srcVerts = model_->bindVertices.empty() ? model_->vertices : model_->bindVertices;
    glBufferData(GL_ARRAY_BUFFER,
                 GLsizeiptr(srcVerts.size() * sizeof(ModelVertex)),
                 srcVerts.data(),
                 useSkinning ? GL_DYNAMIC_DRAW : GL_STATIC_DRAW);

    glGenBuffers(1, &ibo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 GLsizeiptr(model_->indices.size() * sizeof(std::uint32_t)),
                 model_->indices.data(),
                 GL_STATIC_DRAW);

    // Attributes
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(ModelVertex), (void*)offsetof(ModelVertex, px));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(ModelVertex), (void*)offsetof(ModelVertex, nx));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(ModelVertex), (void*)offsetof(ModelVertex, u));

    glBindVertexArray(0);

    gpuSubmeshes_.reserve(model_->subMeshes.size());
    for (const auto& sm : model_->subMeshes)
    {
        GpuSubmesh g;
        g.indexOffset = sm.indexOffset;
        g.indexCount = sm.indexCount;
        g.materialId = sm.materialId;
        gpuSubmeshes_.push_back(g);
    }

    if (placeholderTex_ == 0)
        placeholderTex_ = createPlaceholderTexture();

    if (!modelPath_.isEmpty())
    {
        GpuCacheEntry entry;
        entry.vao = vao_;
        entry.vbo = vbo_;
        entry.ibo = ibo_;
        entry.submeshes = gpuSubmeshes_;
        gpuCache_.insert(modelPath_, entry);
    }
}

GLModelView::TextureResolve GLModelView::resolveTexturePath(const std::string& mdxPath) const
{
    TextureResolve res;
    if (assetRoot_.isEmpty())
        return res;

    QString p = QString::fromStdString(mdxPath).trimmed();
    if (p.isEmpty()) return res;

    p = normPath(p);
    const QString lower = p.toLower();
    if (lower.startsWith("war3mapimported/"))
        p = p.mid(QString("war3mapimported/").length());

    auto addRelCandidate = [&](const QString& rel)
    {
        if (!rel.isEmpty() && !res.vfsCandidates.contains(rel))
            res.vfsCandidates.append(rel);
    };

    auto tryPath = [&](const QString& base, const QString& rel, const QString& source) -> bool
    {
        const QString candidate = QDir(base).filePath(rel);
        res.attempts.append(candidate);
        if (QFileInfo::exists(candidate))
        {
            res.path = candidate;
            res.source = source;
            return true;
        }
        return false;
    };

    // Absolute path?
    if (QFileInfo(p).isAbsolute())
    {
        res.attempts.append(p);
    }
    if (QFileInfo(p).isAbsolute() && QFileInfo::exists(p))
    {
        res.path = p;
        res.source = "absolute";
        return res;
    }

    const QString baseName = QFileInfo(p).fileName();
    if (baseName.isEmpty())
        return res;

    // 1) Same directory as model
    if (!modelDir_.isEmpty())
    {
        if (tryPath(modelDir_, p, "model-dir"))
            return res;
        if (tryPath(modelDir_, baseName, "model-dir-basename"))
            return res;
    }

    // 2) Asset root original path
    if (tryPath(assetRoot_, p, "asset-root"))
        return res;
    addRelCandidate(p);

    // 3) Common Warcraft III folders (within asset root)
    const QStringList commonDirs = {
        "Textures",
        "ReplaceableTextures/TeamColor",
        "ReplaceableTextures/TeamGlow",
        "ReplaceableTextures",
        "Units",
        "Buildings",
        "Doodads",
        "Environment",
        "UI",
        "Abilities",
        "Splats",
        "Terrain"
    };
    for (const auto& dir : commonDirs)
    {
        if (tryPath(assetRoot_, dir + "/" + baseName, "common:" + dir))
            return res;
        if (tryPath(assetRoot_, dir + "/" + p, "common:" + dir))
            return res;
        addRelCandidate(dir + "/" + baseName);
        addRelCandidate(dir + "/" + p);
    }

    // 4) war3mapImported folder
    if (tryPath(assetRoot_, "war3mapImported/" + baseName, "war3mapImported"))
        return res;
    if (tryPath(assetRoot_, "war3mapImported/" + p, "war3mapImported"))
        return res;
    addRelCandidate("war3mapImported/" + baseName);
    addRelCandidate("war3mapImported/" + p);

    // 5) Basename search (first hit)
    QDirIterator it(assetRoot_, QStringList() << baseName, QDir::Files, QDirIterator::Subdirectories);
    if (it.hasNext())
    {
        res.path = it.next();
        res.attempts.append(res.path);
        res.source = "basename-search";
        return res;
    }

    return res;
}

GLuint GLModelView::createPlaceholderTexture()
{
    // 2x2 checker (pink/black)
    const unsigned char pixels[16] = {
        255,  0,255,255,   0,  0,  0,255,
          0,  0,  0,255, 255,  0,255,255,
    };

    GLuint tex = 0;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
    glBindTexture(GL_TEXTURE_2D, 0);
    return tex;
}

GLuint GLModelView::getOrCreateTexture(std::uint32_t textureId)
{
    if (!model_)
        return placeholderTex_;

    auto it = textureCache_.find(textureId);
    if (it != textureCache_.end() && it->second.valid)
        return it->second.id;

    TextureHandle handle;
    handle.id = placeholderTex_;
    handle.valid = true;

    if (textureId < model_->textures.size())
    {
        const auto& tex = model_->textures[textureId];

        if (tex.replaceableId == 1)
        {
            if (teamColorTex_ == 0)
            {
                const unsigned char pixels[4] = { 20, 120, 255, 255 };
                glGenTextures(1, &teamColorTex_);
                glBindTexture(GL_TEXTURE_2D, teamColorTex_);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
                glBindTexture(GL_TEXTURE_2D, 0);
            }
            handle.id = teamColorTex_;
            handle.path = "ReplaceableTextures/TeamColor";
            handle.source = "replaceable:TeamColor";
            textureCache_[textureId] = handle;
            LogSink::instance().log(QString("Texture %1 replaceable TeamColor").arg(textureId));
            return handle.id;
        }
        if (tex.replaceableId == 2)
        {
            if (teamGlowTex_ == 0)
            {
                const unsigned char pixels[4] = { 255, 200, 40, 255 };
                glGenTextures(1, &teamGlowTex_);
                glBindTexture(GL_TEXTURE_2D, teamGlowTex_);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
                glBindTexture(GL_TEXTURE_2D, 0);
            }
            handle.id = teamGlowTex_;
            handle.path = "ReplaceableTextures/TeamGlow";
            handle.source = "replaceable:TeamGlow";
            textureCache_[textureId] = handle;
            LogSink::instance().log(QString("Texture %1 replaceable TeamGlow").arg(textureId));
            return handle.id;
        }

        // Replaceable textures (team color, etc.) aren't resolved here.
        if (!tex.fileName.empty())
        {
            const auto resolved = resolveTexturePath(tex.fileName);
            QStringList attempts = resolved.attempts;
            if (!resolved.path.isEmpty())
            {
                QImage img;
                QString err;
                const QString ext = QFileInfo(resolved.path).suffix().toLower();
                bool ok = false;

                if (ext == "blp")
                {
                    ok = BlpLoader::LoadBlpToImageCached(resolved.path, &img, &err);
                }
                else
                {
                    ok = img.load(resolved.path);
                    if (!ok) err = "Qt failed to load image.";
                    if (ok && img.format() != QImage::Format_RGBA8888)
                        img = img.convertToFormat(QImage::Format_RGBA8888);
                }

                if (ok && !img.isNull())
                {
                      GLuint gltex = 0;
                    glGenTextures(1, &gltex);
                    glBindTexture(GL_TEXTURE_2D, gltex);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

                    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, img.width(), img.height(), 0,
                                 GL_RGBA, GL_UNSIGNED_BYTE, img.constBits());
                    glGenerateMipmap(GL_TEXTURE_2D);

                    glBindTexture(GL_TEXTURE_2D, 0);

                    handle.id = gltex;
                    handle.path = resolved.path;
                    handle.source = resolved.source;
                    LogSink::instance().log(QString("Texture %1 hit %2 -> %3")
                                                .arg(textureId)
                                                .arg(handle.source)
                                                .arg(handle.path));
                }
                else
                {
                    LogSink::instance().log(QString("Texture %1 failed to load %2 | %3")
                                                .arg(textureId)
                                                .arg(resolved.path)
                                                .arg(err));
                    recordMissingTexture(QString::fromStdString(tex.fileName), attempts);
                }
            }
            else if (vfs_)
            {
                QImage img;
                QString err;
                bool ok = false;
                QString source;
                QString foundPath;

                for (const auto& candidate : resolved.vfsCandidates)
                {
                    const QString attempt = QString("mpq:%1").arg(candidate);
                    attempts.append(attempt);

                    const QByteArray bytes = vfs_->readAll(candidate);
                    if (bytes.isEmpty())
                        continue;

                    source = vfs_->resolveDebugInfo(candidate);
                    foundPath = candidate;
                    const QString ext = QFileInfo(candidate).suffix().toLower();
                    if (ext == "blp" || ext.isEmpty())
                        ok = BlpLoader::LoadBlpToImageFromBytes(bytes, &img, &err);
                    else
                    {
                        ok = img.loadFromData(bytes);
                        if (!ok) err = "Qt failed to load image bytes.";
                        if (ok && img.format() != QImage::Format_RGBA8888)
                            img = img.convertToFormat(QImage::Format_RGBA8888);
                    }
                    if (ok)
                        break;
                }

                if (ok && !img.isNull())
                {
                      GLuint gltex = 0;
                    glGenTextures(1, &gltex);
                    glBindTexture(GL_TEXTURE_2D, gltex);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

                    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, img.width(), img.height(), 0,
                                 GL_RGBA, GL_UNSIGNED_BYTE, img.constBits());
                    glGenerateMipmap(GL_TEXTURE_2D);

                    glBindTexture(GL_TEXTURE_2D, 0);

                    handle.id = gltex;
                    handle.path = foundPath;
                    handle.source = source.isEmpty() ? "mpq" : source;
                    LogSink::instance().log(QString("Texture %1 hit %2 -> %3")
                                                .arg(textureId)
                                                .arg(handle.source)
                                                .arg(handle.path));
                }
                else
                {
                    LogSink::instance().log(QString("Texture %1 not found in MPQ: %2")
                                                .arg(textureId)
                                                .arg(QString::fromStdString(tex.fileName)));
                    recordMissingTexture(QString::fromStdString(tex.fileName), attempts);
                }
            }
            else
            {
                LogSink::instance().log(QString("Texture %1 not found: %2")
                                            .arg(textureId)
                                            .arg(QString::fromStdString(tex.fileName)));
                recordMissingTexture(QString::fromStdString(tex.fileName), attempts);
            }
        }
    }

    textureCache_[textureId] = handle;
    return handle.id;
}
