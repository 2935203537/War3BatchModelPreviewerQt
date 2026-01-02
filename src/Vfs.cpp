#include "Vfs.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>

#include "LogSink.h"

#include <StormLib.h>

DiskVfs::DiskVfs(QString rootPath)
    : root_(std::move(rootPath))
{
}

void DiskVfs::setRootPath(const QString& rootPath)
{
    root_ = rootPath;
}

QString DiskVfs::rootPath() const
{
    return root_;
}

bool DiskVfs::exists(const QString& path) const
{
    if (root_.isEmpty())
        return false;
    const QString candidate = QDir(root_).filePath(path);
    return QFileInfo::exists(candidate);
}

QByteArray DiskVfs::readAll(const QString& path) const
{
    if (root_.isEmpty())
        return {};
    const QString candidate = QDir(root_).filePath(path);
    QFile f(candidate);
    if (!f.open(QIODevice::ReadOnly))
        return {};
    return f.readAll();
}

QString DiskVfs::resolveDebugInfo(const QString& path) const
{
    if (root_.isEmpty())
        return {};
    const QString candidate = QDir(root_).filePath(path);
    if (QFileInfo::exists(candidate))
        return QString("disk:%1").arg(candidate);
    return {};
}

MpqVfs::MpqVfs() = default;

MpqVfs::~MpqVfs()
{
    for (auto& a : archives_)
    {
        if (a.handle)
            SFileCloseArchive(a.handle);
        a.handle = nullptr;
    }
}

QString MpqVfs::normalizePath(const QString& path) const
{
    QString p = path;
    p.replace('/', '\\');
    if (p.startsWith("\\"))
        p = p.mid(1);
    return p;
}

bool MpqVfs::mountWar3Root(const QString& rootPath)
{
    for (auto& a : archives_)
    {
        if (a.handle)
            SFileCloseArchive(a.handle);
        a.handle = nullptr;
    }
    archives_.clear();

    const QStringList names = {
        "War3.mpq",
        "War3x.mpq",
        "War3xLocal.mpq",
        "War3Patch.mpq"
    };

    for (const auto& name : names)
    {
        const QString full = QDir(rootPath).filePath(name);
        if (!QFileInfo::exists(full))
            continue;

        HANDLE h = nullptr;
#ifdef UNICODE
        const auto* fullPath = reinterpret_cast<const wchar_t*>(full.utf16());
#else
        const QByteArray fullBytes = QFile::encodeName(full);
        const auto* fullPath = fullBytes.constData();
#endif
        if (SFileOpenArchive(fullPath, 0, MPQ_OPEN_READ_ONLY, &h))
        {
            Archive a;
            a.handle = h;
            a.path = full;
            archives_.push_back(a);
            LogSink::instance().log(QString("MPQ mounted: %1").arg(full));
        }
        else
        {
            LogSink::instance().log(QString("MPQ mount failed: %1").arg(full));
        }
    }

    return !archives_.empty();
}

int MpqVfs::mountedCount() const
{
    return int(archives_.size());
}

QStringList MpqVfs::mountedArchives() const
{
    QStringList out;
    for (const auto& a : archives_)
        out << a.path;
    return out;
}

bool MpqVfs::openFileFromArchives(const QString& normPath, const QString& altPath, void** outFileHandle, QString* outArchive) const
{
    const QByteArray normBytes = normPath.toUtf8();
    const QByteArray altBytes = altPath.toUtf8();
    for (auto it = archives_.rbegin(); it != archives_.rend(); ++it)
    {
        HANDLE hFile = nullptr;
        if (SFileOpenFileEx(it->handle, normBytes.constData(), SFILE_OPEN_FROM_MPQ, &hFile))
        {
            if (outFileHandle) *outFileHandle = hFile;
            if (outArchive) *outArchive = it->path;
            return true;
        }
        if (!altPath.isEmpty() && altPath != normPath)
        {
            if (SFileOpenFileEx(it->handle, altBytes.constData(), SFILE_OPEN_FROM_MPQ, &hFile))
            {
                if (outFileHandle) *outFileHandle = hFile;
                if (outArchive) *outArchive = it->path;
                return true;
            }
        }
    }
    return false;
}

bool MpqVfs::exists(const QString& path) const
{
    const QString norm = normalizePath(path);
    const QString alt = norm.toUpper();

    void* hFile = nullptr;
    QString archive;
    const bool ok = openFileFromArchives(norm, alt, &hFile, &archive);
    if (ok && hFile)
        SFileCloseFile(hFile);
    return ok;
}

QByteArray MpqVfs::readAll(const QString& path) const
{
    const QString norm = normalizePath(path);
    const QString alt = norm.toUpper();

    HANDLE hFile = nullptr;
    QString archive;
    if (!openFileFromArchives(norm, alt, (void**)&hFile, &archive))
    {
        LogSink::instance().log(QString("MPQ miss: %1 | tried %2 / %3").arg(path, norm, alt));
        return {};
    }

    DWORD fileSize = SFileGetFileSize(hFile, nullptr);
    if (fileSize == 0 || fileSize == SFILE_INVALID_SIZE)
    {
        SFileCloseFile(hFile);
        return {};
    }

    QByteArray data;
    data.resize(int(fileSize));
    DWORD read = 0;
    const bool ok = SFileReadFile(hFile, data.data(), fileSize, &read, nullptr);
    SFileCloseFile(hFile);
    if (!ok || read != fileSize)
        return {};

    return data;
}

QString MpqVfs::resolveDebugInfo(const QString& path) const
{
    const QString norm = normalizePath(path);
    const QString alt = norm.toUpper();
    HANDLE hFile = nullptr;
    QString archive;
    if (openFileFromArchives(norm, alt, (void**)&hFile, &archive))
    {
        if (hFile)
            SFileCloseFile(hFile);
        return QString("mpq:%1").arg(archive);
    }
    return {};
}

void CompositeVfs::add(const std::shared_ptr<IVfs>& vfs)
{
    if (vfs)
        list_.push_back(vfs);
}

bool CompositeVfs::exists(const QString& path) const
{
    for (const auto& vfs : list_)
    {
        if (vfs && vfs->exists(path))
            return true;
    }
    return false;
}

QByteArray CompositeVfs::readAll(const QString& path) const
{
    for (const auto& vfs : list_)
    {
        if (!vfs)
            continue;
        const QByteArray data = vfs->readAll(path);
        if (!data.isEmpty())
            return data;
    }
    return {};
}

QString CompositeVfs::resolveDebugInfo(const QString& path) const
{
    for (const auto& vfs : list_)
    {
        if (!vfs)
            continue;
        const QString info = vfs->resolveDebugInfo(path);
        if (!info.isEmpty())
            return info;
    }
    return {};
}
