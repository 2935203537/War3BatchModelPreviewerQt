#pragma once

#include <QFile>
#include <QMutex>
#include <QObject>
#include <QTextStream>

class LogSink final : public QObject
{
    Q_OBJECT
public:
    static LogSink& instance();

    void init(const QString& logPath);
    void log(const QString& message);

signals:
    void messageAdded(const QString& message);

private:
    explicit LogSink(QObject* parent = nullptr);
    Q_DISABLE_COPY_MOVE(LogSink)

    QFile file_;
    QTextStream stream_;
    QMutex mutex_;
};
