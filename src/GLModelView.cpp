#include "GLModelView.h"

#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QVector3D>
#include <QtMath>

#include <vector>
#include <cstddef>
#include <cstring>
#include <algorithm>
#include <cmath>

namespace
{
    static unsigned int CompileShader(QOpenGLFunctions_3_3_Core* f, GLenum type, const char* src, QString* outErr)
    {
        unsigned int s = f->glCreateShader(type);
        f->glShaderSource(s, 1, &src, nullptr);
        f->glCompileShader(s);
        int ok = 0;
        f->glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
        if (!ok)
        {
            int len = 0;
            f->glGetShaderiv(s, GL_INFO_LOG_LENGTH, &len);
            std::vector<char> buf(static_cast<std::size_t>(len > 1 ? len : 1));
            f->glGetShaderInfoLog(s, len, nullptr, buf.data());
            if (outErr) *outErr = QString::fromUtf8(buf.data());
            f->glDeleteShader(s);
            return 0;
        }
        return s;
    }

    static unsigned int LinkProgram(QOpenGLFunctions_3_3_Core* f, unsigned int vs, unsigned int fs, QString* outErr)
    {
        unsigned int p = f->glCreateProgram();
        f->glAttachShader(p, vs);
        f->glAttachShader(p, fs);
        f->glLinkProgram(p);
        int ok = 0;
        f->glGetProgramiv(p, GL_LINK_STATUS, &ok);
        if (!ok)
        {
            int len = 0;
            f->glGetProgramiv(p, GL_INFO_LOG_LENGTH, &len);
            std::vector<char> buf(static_cast<std::size_t>(len > 1 ? len : 1));
            f->glGetProgramInfoLog(p, len, nullptr, buf.data());
            if (outErr) *outErr = QString::fromUtf8(buf.data());
            f->glDeleteProgram(p);
            return 0;
        }
        return p;
    }
}

GLModelView::GLModelView(QWidget* parent)
    : QOpenGLWidget(parent)
{
    setFocusPolicy(Qt::StrongFocus);
}

GLModelView::~GLModelView()
{
    makeCurrent();
    destroyGlObjects();
    doneCurrent();
}

void GLModelView::setModel(const std::optional<ModelData>& model, const QString& label)
{
    model_ = model;
    label_ = label;
    gpuDirty_ = true;

    if (model_)
        frameModel();

    updateStatus();
    update();
}

void GLModelView::frameModel()
{
    if (!model_ || !model_->hasBounds) return;

    const float* mn = model_->boundsMin;
    const float* mx = model_->boundsMax;

    QVector3D center(
        (mn[0] + mx[0]) * 0.5f,
        (mn[1] + mx[1]) * 0.5f,
        (mn[2] + mx[2]) * 0.5f
    );

    const float dx = mx[0] - mn[0];
    const float dy = mx[1] - mn[1];
    const float dz = mx[2] - mn[2];
    const float radius = 0.5f * std::sqrt(dx*dx + dy*dy + dz*dz);

    cam_.target = center;
    cam_.distance = std::max(10.0f, radius * 2.5f);
}

void GLModelView::initializeGL()
{
    initializeOpenGLFunctions();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    const char* vsSrc = R"GLSL(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec3 aNrm;

        uniform mat4 uMVP;
        uniform mat4 uModel;

        out vec3 vNrm;

        void main() {
            gl_Position = uMVP * vec4(aPos, 1.0);
            // Transform normal by model (no non-uniform scaling here)
            vNrm = mat3(uModel) * aNrm;
        }
    )GLSL";

    const char* fsSrc = R"GLSL(
        #version 330 core
        in vec3 vNrm;

        uniform vec3 uLightDir;

        out vec4 FragColor;

        void main() {
            vec3 n = normalize(vNrm);
            float ndl = max(dot(n, normalize(uLightDir)), 0.0);
            float ambient = 0.25;
            float diff = 0.75 * ndl;
            float shade = ambient + diff;
            FragColor = vec4(vec3(shade), 1.0);
        }
    )GLSL";

    QString err;
    unsigned int vs = CompileShader(this, GL_VERTEX_SHADER, vsSrc, &err);
    if (!vs) { emit statusTextChanged("Shader compile error: " + err); return; }
    unsigned int fs = CompileShader(this, GL_FRAGMENT_SHADER, fsSrc, &err);
    if (!fs) { emit statusTextChanged("Shader compile error: " + err); glDeleteShader(vs); return; }
    prog_ = LinkProgram(this, vs, fs, &err);
    glDeleteShader(vs);
    glDeleteShader(fs);

    if (!prog_) { emit statusTextChanged("Shader link error: " + err); return; }

    locMvp_ = glGetUniformLocation(prog_, "uMVP");
    locModel_ = glGetUniformLocation(prog_, "uModel");
    locLightDir_ = glGetUniformLocation(prog_, "uLightDir");

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glGenBuffers(1, &ebo_);

    ensureGpuBuffers();
}

void GLModelView::resizeGL(int w, int h)
{
    projection_.setToIdentity();
    const float aspect = (h > 0) ? (static_cast<float>(w) / static_cast<float>(h)) : 1.0f;
    projection_.perspective(45.0f, aspect, 0.1f, 100000.0f);
}

void GLModelView::ensureGpuBuffers()
{
    if (!prog_ || !vao_ || !vbo_ || !ebo_) return;

    if (!gpuDirty_) return;
    gpuDirty_ = false;

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);

    if (model_ && !model_->vertices.empty() && !model_->indices.empty())
    {
        glBufferData(GL_ARRAY_BUFFER,
            static_cast<GLsizeiptr>(model_->vertices.size() * sizeof(ModelVertex)),
            model_->vertices.data(),
            GL_STATIC_DRAW);

        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
            static_cast<GLsizeiptr>(model_->indices.size() * sizeof(std::uint32_t)),
            model_->indices.data(),
            GL_STATIC_DRAW);
    }
    else
    {
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_STATIC_DRAW);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, 0, nullptr, GL_STATIC_DRAW);
    }

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(ModelVertex), (void*)offsetof(ModelVertex, px));

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(ModelVertex), (void*)offsetof(ModelVertex, nx));

    glBindVertexArray(0);
}

void GLModelView::paintGL()
{
    glClearColor(0.08f, 0.08f, 0.09f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    ensureGpuBuffers();

    if (!prog_ || !model_ || model_->indices.empty() || model_->vertices.empty())
        return;

    // Camera position (orbit around target)
    const float yawRad = qDegreesToRadians(cam_.yaw);
    const float pitchRad = qDegreesToRadians(cam_.pitch);

    const float cy = std::cos(yawRad);
    const float sy = std::sin(yawRad);
    const float cp = std::cos(pitchRad);
    const float sp = std::sin(pitchRad);

    QVector3D forward(cp * cy, cp * sy, sp);
    QVector3D eye = cam_.target + forward * cam_.distance;

    QMatrix4x4 view;
    view.lookAt(eye, cam_.target, QVector3D(0, 0, 1));

    QMatrix4x4 modelMat;
    modelMat.setToIdentity();

    QMatrix4x4 mvp = projection_ * view * modelMat;

    glUseProgram(prog_);
    glUniformMatrix4fv(locMvp_, 1, GL_FALSE, mvp.constData());
    glUniformMatrix4fv(locModel_, 1, GL_FALSE, modelMat.constData());

    // Light from "camera-ish" direction
    QVector3D lightDir = (cam_.target - eye).normalized();
    glUniform3f(locLightDir_, lightDir.x(), lightDir.y(), lightDir.z());

    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(model_->indices.size()), GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);

    glUseProgram(0);
}

void GLModelView::mousePressEvent(QMouseEvent* e)
{
    lastMousePos_ = e->pos();
    leftDown_ = (e->buttons() & Qt::LeftButton);
    rightDown_ = (e->buttons() & Qt::RightButton);
}

void GLModelView::mouseMoveEvent(QMouseEvent* e)
{
    const QPoint delta = e->pos() - lastMousePos_;
    lastMousePos_ = e->pos();

    if (leftDown_)
    {
        cam_.yaw   += delta.x() * 0.4f;
        cam_.pitch -= delta.y() * 0.4f;
        cam_.pitch = std::clamp(cam_.pitch, -89.0f, 89.0f);
        update();
    }
    else if (rightDown_)
    {
        // Pan in view plane
        const float panScale = std::max(1.0f, cam_.distance) * 0.002f;

        QVector3D up(0, 0, 1);

        const float yawRad = qDegreesToRadians(cam_.yaw);
        const float pitchRad = qDegreesToRadians(cam_.pitch);
        QVector3D forward(std::cos(pitchRad) * std::cos(yawRad), std::cos(pitchRad) * std::sin(yawRad), std::sin(pitchRad));
        QVector3D right = QVector3D::crossProduct(forward, up).normalized();
        QVector3D realUp = QVector3D::crossProduct(right, forward).normalized();

        cam_.target += (-right * delta.x() + realUp * delta.y()) * panScale;
        update();
    }
}

void GLModelView::wheelEvent(QWheelEvent* e)
{
    const float numDegrees = e->angleDelta().y() / 8.0f;
    const float numSteps = numDegrees / 15.0f;

    const float zoomFactor = std::pow(0.9f, numSteps);
    cam_.distance = std::clamp(cam_.distance * zoomFactor, 1.0f, 100000.0f);

    update();
}

void GLModelView::keyPressEvent(QKeyEvent* e)
{
    if (e->key() == Qt::Key_F)
    {
        frameModel();
        update();
        e->accept();
        return;
    }
    QOpenGLWidget::keyPressEvent(e);
}

void GLModelView::destroyGlObjects()
{
    if (ebo_) { glDeleteBuffers(1, &ebo_); ebo_ = 0; }
    if (vbo_) { glDeleteBuffers(1, &vbo_); vbo_ = 0; }
    if (vao_) { glDeleteVertexArrays(1, &vao_); vao_ = 0; }
    if (prog_) { glDeleteProgram(prog_); prog_ = 0; }
}

void GLModelView::updateStatus()
{
    if (!model_)
    {
        emit statusTextChanged("No model loaded.");
        return;
    }

    const int v = static_cast<int>(model_->vertices.size());
    const int t = static_cast<int>(model_->indices.size() / 3);
    emit statusTextChanged(QString("%1 | %2 verts | %3 tris").arg(label_).arg(v).arg(t));
}
