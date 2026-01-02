#include "GLModelView.h"

#include <QMouseEvent>
#include <QWheelEvent>
#include <QFileInfo>
#include <QDir>
#include <QDirIterator>
#include <QImage>

#include <cmath>

#include <cstddef>

#include "BlpLoader.h"

namespace
{
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
}

GLModelView::~GLModelView()
{
    makeCurrent();
    clearGpuResources();
    doneCurrent();
}

void GLModelView::setAssetRoot(const QString& assetRoot)
{
    assetRoot_ = assetRoot;
}

void GLModelView::setModel(std::optional<ModelData> model, const QString& displayName)
{
    displayName_ = displayName;
    model_ = std::move(model);

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
        distance_ = clampf(modelRadius_ * 2.5f, 1.0f, 250.0f);
    }
    else
    {
        modelCenter_ = QVector3D(0, 0, 0);
        modelRadius_ = 1.0f;
        distance_ = 6.0f;
    }

    emit statusTextChanged(model_ ? QString("%1 | %2 verts, %3 tris, %4 submeshes")
                                       .arg(displayName_)
                                       .arg(model_->vertices.size())
                                       .arg(model_->indices.size() / 3)
                                       .arg(model_->subMeshes.size())
                                 : QString("%1 | <no geometry>").arg(displayName_));

    update();
}

void GLModelView::initializeGL()
{
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);

    // Warcraft III MDX was built for a D3D-style renderer where clockwise is typically the front face.
    glFrontFace(GL_CW);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    // Basic shader: textured + lambert, with optional alpha test.
    program_.addShaderFromSourceCode(QOpenGLShader::Vertex, R"GLSL(
        #version 330 core
        layout(location=0) in vec3 aPos;
        layout(location=1) in vec3 aNrm;
        layout(location=2) in vec2 aUV;

        uniform mat4 uMVP;
        uniform mat4 uModel;
        uniform mat3 uNormalMat;

        out vec3 vNrm;
        out vec2 vUV;

        void main(){
            gl_Position = uMVP * vec4(aPos, 1.0);
            vNrm = normalize(uNormalMat * aNrm);
            vUV = aUV;
        }
    )GLSL");

    program_.addShaderFromSourceCode(QOpenGLShader::Fragment, R"GLSL(
        #version 330 core
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
    )GLSL");

    programReady_ = program_.link();
    if (!programReady_)
        emit statusTextChanged("Shader link failed: " + program_.log());

    rebuildGpuBuffers();
}

void GLModelView::resizeGL(int w, int h)
{
    proj_.setToIdentity();
    proj_.perspective(45.0f, float(w) / float(std::max(h, 1)), 0.01f, 2000.0f);
}

void GLModelView::paintGL()
{
    glViewport(0, 0, width(), height());
    glClearColor(0.10f, 0.10f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (!programReady_ || !model_ || vao_ == 0 || model_->indices.empty())
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

    QMatrix4x4 mvp = proj_ * view * modelM;
    QMatrix3x3 normalMat = modelM.normalMatrix();

    program_.bind();
    program_.setUniformValue("uMVP", mvp);
    program_.setUniformValue("uModel", modelM);
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

    // Pass 2: blended materials (depth write usually off if flagged)
    // Basic sort by priority plane to reduce obvious artifacts.
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
    if (ibo_) { glDeleteBuffers(1, &ibo_); ibo_ = 0; }
    if (vbo_) { glDeleteBuffers(1, &vbo_); vbo_ = 0; }
    if (vao_) { glDeleteVertexArrays(1, &vao_); vao_ = 0; }

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

    gpuSubmeshes_.clear();
}

void GLModelView::rebuildGpuBuffers()
{
    clearGpuResources();

    if (!model_ || model_->vertices.empty() || model_->indices.empty())
        return;

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

    // Placeholder texture
    placeholderTex_ = createPlaceholderTexture();
}

QString GLModelView::resolveTexturePath(const std::string& mdxPath) const
{
    if (assetRoot_.isEmpty())
        return {};

    QString p = QString::fromStdString(mdxPath).trimmed();
    if (p.isEmpty()) return {};

    p = normPath(p);

    // Some models store "war3mapImported/..." when bundled in maps; try stripping.
    const QString lower = p.toLower();
    if (lower.startsWith("war3mapimported/"))
        p = p.mid(QString("war3mapImported/").length());

    const QString direct = QDir(assetRoot_).filePath(p);
    if (QFileInfo::exists(direct))
        return direct;

    // Try as basename search (first hit)
    const QString base = QFileInfo(p).fileName();
    if (base.isEmpty()) return {};

    QDirIterator it(assetRoot_, QStringList() << base, QDir::Files, QDirIterator::Subdirectories);
    if (it.hasNext())
        return it.next();

    return {};
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

        // Replaceable textures (team color, etc.) aren't resolved here.
        if (!tex.fileName.empty())
        {
            const QString resolved = resolveTexturePath(tex.fileName);
            if (!resolved.isEmpty())
            {
                QImage img;
                QString err;
                const QString ext = QFileInfo(resolved).suffix().toLower();
                bool ok = false;

                if (ext == "blp")
                {
                    ok = BlpLoader::LoadBlpToImage(resolved, &img, &err);
                }
                else
                {
                    ok = img.load(resolved);
                    if (!ok) err = "Qt failed to load image.";
                    if (ok && img.format() != QImage::Format_RGBA8888)
                        img = img.convertToFormat(QImage::Format_RGBA8888);
                }

                if (ok && !img.isNull())
                {
                    // Flip vertically for OpenGL UV origin.
                    img = img.mirrored(false, true);

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
                    handle.valid = true;
                }
                else
                {
                    emit statusTextChanged(QString("%1 | texture load failed: %2 (%3)")
                                                .arg(displayName_)
                                                .arg(QFileInfo(resolved).fileName())
                                                .arg(err));
                }
            }
        }
    }

    textureCache_[textureId] = handle;
    return handle.id;
}
