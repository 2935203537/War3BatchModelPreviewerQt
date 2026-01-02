#include "LogSink.h"

#include <QDateTime>
#include <QDir>

LogSink& LogSink::instance()
{
    static LogSink sink;
    return sink;
}

LogSink::LogSink(QObject* parent)
    : QObject(parent)
{
}

void LogSink::init(const QString& logPath)
{
    QMutexLocker lock(&mutex_);
    if (file_.isOpen())
        file_.close();

    file_.setFileName(logPath);
    if (file_.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
        stream_.setDevice(&file_);
}

void LogSink::log(const QString& message)
{
    const QString line = QString("[%1] %2")
                             .arg(QDateTime::currentDateTime().toString("HH:mm:ss"))
                             .arg(message);
    {
        QMutexLocker lock(&mutex_);
        if (stream_.device())
        {
            stream_ << line << "\n";
            stream_.flush();
        }
    }
    emit messageAdded(line);
}
