#include <QApplication>
#include <QDir>
#include <QDirIterator>
#include <QDebug>
#include <QFile>
#include <QTextStream>
#include <QSurfaceFormat>

#include "MainWindow.h"
#include "MdxLoader.h"
#include "LogSink.h"

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

    QDir(QDir::current()).mkpath("logs");
    LogSink::instance().init(QDir(QDir::current()).filePath("logs/latest.log"));

    if (qEnvironmentVariableIsSet("MDX_DEBUG_LOAD"))
    {
        QFile logFile;
        QTextStream logStream;
        const QString logPath = qEnvironmentVariable("MDX_DEBUG_LOG");
        if (!logPath.isEmpty())
        {
            logFile.setFileName(logPath);
            if (logFile.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
                logStream.setDevice(&logFile);
        }

        auto logLine = [&](const QString& line, bool warn)
        {
            if (warn)
                qWarning().noquote() << line;
            else
                qDebug().noquote() << line;
            if (logStream.device())
            {
                logStream << line << "\n";
                logStream.flush();
            }
        };

        QDir cwd(QDir::currentPath());
        if (cwd.exists("resource"))
        {
            QDir res(cwd.filePath("resource"));
            QDirIterator it(res.absolutePath(),
                            QStringList() << "*.mdx" << "*.MDX",
                            QDir::Files,
                            QDirIterator::Subdirectories);
            while (it.hasNext())
            {
                const QString path = it.next();
                QString err;
                const auto model = MdxLoader::LoadFromFile(path, &err);
                if (!model)
                {
                    logLine(QString("MDX load failed: %1 | %2").arg(path, err), true);
                }
                else
                {
                    std::uint32_t maxIndex = 0;
                    for (std::uint32_t idx : model->indices)
                        maxIndex = std::max(maxIndex, idx);
                    const bool indexOk = model->indices.empty() || (maxIndex < model->vertices.size());

                    logLine(QString("MDX load ok: %1 | verts %2 | tris %3 | submeshes %4 | maxIndex %5 | indexOk %6 | bounds [%7,%8,%9]-[%10,%11,%12]")
                                .arg(path)
                                .arg(model->vertices.size())
                                .arg(model->indices.size() / 3)
                                .arg(model->subMeshes.size())
                                .arg(maxIndex)
                                .arg(indexOk ? "yes" : "no")
                                .arg(model->boundsMin[0], 0, 'f', 3)
                                .arg(model->boundsMin[1], 0, 'f', 3)
                                .arg(model->boundsMin[2], 0, 'f', 3)
                                .arg(model->boundsMax[0], 0, 'f', 3)
                                .arg(model->boundsMax[1], 0, 'f', 3)
                                .arg(model->boundsMax[2], 0, 'f', 3),
                            false);
                }
            }
        }
        else
        {
            logLine(QString("MDX_DEBUG_LOAD set but resource folder not found in %1").arg(cwd.absolutePath()), true);
        }

        if (qEnvironmentVariableIsSet("MDX_DEBUG_EXIT"))
        {
            return 0;
        }
    }

    MainWindow w;
    w.show();

    return app.exec();
}
