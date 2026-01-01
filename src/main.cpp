#include <QApplication>
#include <QSurfaceFormat>

#include "MainWindow.h"

static void ConfigureOpenGL()
{
    // Request a modern core profile. If the system can't provide it,
    // Qt may fall back (e.g., ANGLE). The viewer handles that gracefully.
    QSurfaceFormat fmt;
    fmt.setDepthBufferSize(24);
    fmt.setStencilBufferSize(8);
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setSamples(4);
    QSurfaceFormat::setDefaultFormat(fmt);
}

int main(int argc, char *argv[])
{
    ConfigureOpenGL();

    QApplication app(argc, argv);
    QApplication::setApplicationName("War3 Batch Model Previewer");
    QApplication::setOrganizationName("Local");

    MainWindow w;
    w.show();

    return app.exec();
}
