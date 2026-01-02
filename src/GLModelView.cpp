#include "GLModelView.h"

#include <QMouseEvent>
#include <QWheelEvent>
#include <QFileInfo>
#include <QDir>
#include <QDirIterator>
#include <QImage>
#include <QOpenGLContext>
#include <QQuaternion>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <random>

#include "BlpLoader.h"
#include "LogSink.h"

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

    // MDX Layer shading flags (common ones used for preview)
    constexpr std::uint32_t LAYER_UNSHADED   = 0x1;
    constexpr std::uint32_t LAYER_TWOSIDED   = 0x10;
    constexpr std::uint32_t LAYER_NODEPTH    = 0x40;
    constexpr std::uint32_t LAYER_NODEPTHSET = 0x80;
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

void GLModelView::setPlaybackSpeed(float speed)
{
    playbackSpeed_ = clampf(speed, 0.05f, 10.0f);
}

void GLModelView::setAssetRoot(const QString& assetRoot)
{
    assetRoot_ = assetRoot;
}

void GLModelView::resetView()
{
    yaw_ = 30.0f;
    pitch_ = -25.0f;
    distance_ = clampf(modelRadius_ * 2.5f, 1.0f, 250.0f);
    update();
}

void GLModelView::setModel(std::optional<ModelData> model, const QString& displayName, const QString& filePath)
{
    displayName_ = displayName;
    modelPath_ = filePath;
    modelDir_ = filePath.isEmpty() ? QString() : QFileInfo(filePath).absolutePath();
    model_ = std::move(model);

    localTimeMs_ = 0;
    currentSeq_ = 0;
    frameTimer_.restart();

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

    // Frame model if we have bounds, otherwise fall back.
    if (model_ && model_->hasBounds)
    {
        const auto& m = *model_;
        modelCenter_ = QVector3D(
            (m.boundsMin[0] + m.boundsMax[0]) * 0.5f,
            (m.boundsMin[1] + m.boundsMax[1]) * 0.5f,
            (m.boundsMin[2] + m.boundsMax[2]) * 0.5f
        );
        const QVector3D ext(
            (m.boundsMax[0] - m.boundsMin[0]) * 0.5f,
            (m.boundsMax[1] - m.boundsMin[1]) * 0.5f,
            (m.boundsMax[2] - m.boundsMin[2]) * 0.5f
        );
        modelRadius_ = std::max(0.25f, ext.length());
    }
    else
    {
        modelCenter_ = QVector3D(0, 0, 0);
        modelRadius_ = 1.0f;
    }

    resetView();

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
                [](const QOpenGLDebugMessage& msg){
                    const QString line = QString("GL: [%1] %2 (id=%3)")
                                             .arg(msg.severity())
                                             .arg(msg.message())
                                             .arg(msg.id());
                    LogSink::instance().log(line);
                });
        glLogger_.startLogging(QOpenGLDebugLogger::SynchronousLogging);
        glLogger_.enableMessages();
        LogSink::instance().log("GL debug logger initialized.");
    }

    // Many War3 assets have inconsistent winding. For a preview tool, disable culling
    // so models show up reliably.
    glDisable(GL_CULL_FACE);

    const bool isGles = QOpenGLContext::currentContext()
                            ? QOpenGLContext::currentContext()->isOpenGLES()
                            : false;
    const QString glslHeader = isGles ? "#version 300 es\n" : "#version 330 core\n";
    const QString glslFragPreamble = isGles ? "precision mediump float;\n" : "";

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

    rebuildGpuBuffers();
}

void GLModelView::resizeGL(int w, int h)
{
    proj_.setToIdentity();
    proj_.perspective(45.0f, float(w) / float(std::max(h, 1)), 0.01f, 2000.0f);
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
        const float emissionRate = std::max(0.0f, sampleTrackFloat(e.trackEmissionRate, globalTimeMs, e.emissionRate, *model_));
        const float gravity = sampleTrackFloat(e.trackGravity, globalTimeMs, e.gravity, *model_);
        const float lifespan = std::max(0.01f, sampleTrackFloat(e.trackLifespan, globalTimeMs, e.lifespan, *model_));

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
                    const float sx = randSigned() * e.width;
                    const float sy = randSigned() * e.length;
                    p.pos = pivot + QVector3D(sx, sy, 0.0f);

                    // Direction within latitude (simplified)
                    const float lat = e.latitude;
                    const float ax = randSigned() * lat;
                    const float ay = randSigned() * lat;

                    QVector3D dir(0,0,1);
                    QQuaternion qx = QQuaternion::fromAxisAndAngle(1,0,0, ax * 57.2957795f);
                    QQuaternion qy = QQuaternion::fromAxisAndAngle(0,1,0, ay * 57.2957795f);
                    dir = (qy * qx).rotatedVector(dir);
                    dir.normalize();

                    const float sp = speed + randSigned() * e.variation;
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

void GLModelView::paintGL()
{
    glViewport(0, 0, width(), height());
    glClearColor(0.10f, 0.10f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (!model_)
        return;

    // Camera: orbit around model center
    QMatrix4x4 view;
    view.setToIdentity();
    view.translate(0, 0, -distance_);
    view.rotate(pitch_, 1, 0, 0);
    view.rotate(yaw_, 0, 1, 0);
    view.translate(-modelCenter_);

    QMatrix4x4 modelM;
    modelM.setToIdentity();

    const QMatrix4x4 mvp = proj_ * view * modelM;
    const QMatrix3x3 normalMat = modelM.normalMatrix();

    // --- Draw mesh (if any)
    if (programReady_ && vao_ != 0 && !model_->indices.empty())
    {
        program_.bind();
        program_.setUniformValue("uMVP", mvp);
        program_.setUniformValue("uNormalMat", normalMat);

        glBindVertexArray(vao_);

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
            const bool twoSided = (layer.shadingFlags & LAYER_TWOSIDED) != 0;
            const bool noDepthTest = (layer.shadingFlags & LAYER_NODEPTH) != 0;
            const bool noDepthSet  = (layer.shadingFlags & LAYER_NODEPTHSET) != 0;

            const std::uint32_t filter = layer.filterMode;

            const bool alphaTest = (filter == 1);                 // Transparent
            const bool blended   = (filter == 2 || filter == 3 || filter == 4 || filter == 5 || filter == 6);

            if (transparentPass != blended)
                return;

            if (twoSided) glDisable(GL_CULL_FACE); else glEnable(GL_CULL_FACE);

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

        // Reset to a known baseline before other passes.
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDisable(GL_CULL_FACE);
    }

    // --- Draw particles (PRE2)
    if (particleProgramReady_ && pVao_ != 0 && !model_->emitters2.empty())
    {
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
            if (alphaKey)
            {
                particleProgram_.setUniformValue("uAlphaTest", 1);
                particleProgram_.setUniformValue("uAlphaCutoff", 0.75f);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            }
            else
            {
                particleProgram_.setUniformValue("uAlphaTest", 0);
                particleProgram_.setUniformValue("uAlphaCutoff", 0.0f);
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
        }

        glBindVertexArray(0);

        // Restore defaults
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        glDisable(GL_CULL_FACE);
        particleProgram_.release();
    }
}

void GLModelView::mousePressEvent(QMouseEvent* e)
{
    lastMouse_ = e->pos();
}

void GLModelView::mouseMoveEvent(QMouseEvent* e)
{
    const QPoint delta = e->pos() - lastMouse_;
    lastMouse_ = e->pos();

    if (e->buttons() & Qt::LeftButton)
    {
        yaw_ += delta.x() * 0.4f;
        pitch_ += delta.y() * 0.4f;
        pitch_ = clampf(pitch_, -89.0f, 89.0f);
        update();
    }
}

void GLModelView::wheelEvent(QWheelEvent* e)
{
    const float num = e->angleDelta().y() / 120.0f;
    distance_ *= std::pow(0.90f, num);
    distance_ = clampf(distance_, 0.25f, 2000.0f);
    update();
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
    glBufferData(GL_ARRAY_BUFFER,
                 GLsizeiptr(model_->vertices.size() * sizeof(ModelVertex)),
                 model_->vertices.data(),
                 GL_STATIC_DRAW);

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

    auto tryPath = [&](const QString& base, const QString& rel, const QString& source) -> bool
    {
        const QString candidate = QDir(base).filePath(rel);
        if (QFileInfo::exists(candidate))
        {
            res.path = candidate;
            res.source = source;
            return true;
        }
        return false;
    };

    // Absolute path?
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

    // 2) Common Warcraft III folders (within asset root)
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
    }

    // 3) war3mapImported folder
    if (tryPath(assetRoot_, "war3mapImported/" + baseName, "war3mapImported"))
        return res;
    if (tryPath(assetRoot_, "war3mapImported/" + p, "war3mapImported"))
        return res;

    // 4) Direct path under asset root
    if (tryPath(assetRoot_, p, "asset-root"))
        return res;

    // 5) Basename search (first hit)
    QDirIterator it(assetRoot_, QStringList() << baseName, QDir::Files, QDirIterator::Subdirectories);
    if (it.hasNext())
    {
        res.path = it.next();
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
                    // Flip vertically for OpenGL UV origin.
                    img = img.flipped(Qt::Vertical);

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
                }
            }
            else
            {
                LogSink::instance().log(QString("Texture %1 not found: %2")
                                            .arg(textureId)
                                            .arg(QString::fromStdString(tex.fileName)));
            }
        }
    }

    textureCache_[textureId] = handle;
    return handle.id;
}
