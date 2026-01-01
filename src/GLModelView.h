#pragma once
#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QMatrix4x4>
#include <QPoint>
#include <QVector3D>
#include <optional>

#include "ModelData.h"

class GLModelView final : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
    Q_OBJECT
public:
    explicit GLModelView(QWidget* parent = nullptr);
    ~GLModelView() override;

    void setModel(const std::optional<ModelData>& model, const QString& label);
    void frameModel();

signals:
    void statusTextChanged(const QString& text);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;
    void keyPressEvent(QKeyEvent* e) override;

private:
    void destroyGlObjects();
    void ensureGpuBuffers();
    void updateStatus();

    struct Camera
    {
        float yaw = 45.0f;
        float pitch = 25.0f;
        float distance = 200.0f;
        QVector3D target = QVector3D(0, 0, 0);
    } cam_;

    QPoint lastMousePos_;
    bool leftDown_ = false;
    bool rightDown_ = false;

    // GL objects
    unsigned int vao_ = 0;
    unsigned int vbo_ = 0;
    unsigned int ebo_ = 0;
    unsigned int prog_ = 0;

    int locMvp_ = -1;
    int locModel_ = -1;
    int locLightDir_ = -1;

    std::optional<ModelData> model_;
    QString label_;
    bool gpuDirty_ = true;

    QMatrix4x4 projection_;
};
