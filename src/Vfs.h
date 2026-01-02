#pragma once

#include <QByteArray>
#include <QString>
#include <QStringList>
#include <memory>
#include <vector>

class IVfs
{
public:
    virtual ~IVfs() = default;
    virtual bool exists(const QString& path) const = 0;
    virtual QByteArray readAll(const QString& path) const = 0;
    virtual QString resolveDebugInfo(const QString& path) const = 0;
};

class DiskVfs final : public IVfs
{
public:
    explicit DiskVfs(QString rootPath);
    void setRootPath(const QString& rootPath);
    QString rootPath() const;

    bool exists(const QString& path) const override;
    QByteArray readAll(const QString& path) const override;
    QString resolveDebugInfo(const QString& path) const override;

private:
    QString root_;
};

class MpqVfs final : public IVfs
{
public:
    MpqVfs();
    ~MpqVfs() override;

    bool mountWar3Root(const QString& rootPath);
    int mountedCount() const;
    QStringList mountedArchives() const;

    bool exists(const QString& path) const override;
    QByteArray readAll(const QString& path) const override;
    QString resolveDebugInfo(const QString& path) const override;

private:
    struct Archive
    {
        void* handle = nullptr;
        QString path;
    };

    QString normalizePath(const QString& path) const;
    QStringList buildCandidatePaths(const QString& path) const;
    bool openFileFromArchives(const QStringList& candidates, void** outFileHandle, QString* outArchive) const;

    std::vector<Archive> archives_;
};

class CompositeVfs final : public IVfs
{
public:
    void add(const std::shared_ptr<IVfs>& vfs);

    bool exists(const QString& path) const override;
    QByteArray readAll(const QString& path) const override;
    QString resolveDebugInfo(const QString& path) const override;

private:
    std::vector<std::shared_ptr<IVfs>> list_;
};
