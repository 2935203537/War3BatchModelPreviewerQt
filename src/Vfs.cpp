#include "Vfs.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>

#include "LogSink.h"

#include <StormLib.h>
#include <windows.h>

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

QStringList MpqVfs::buildCandidatePaths(const QString& path) const
{
    const QString norm = normalizePath(path);
    QStringList candidates;
    auto add = [&](const QString& s)
    {
        if (!s.isEmpty() && !candidates.contains(s))
            candidates.append(s);
    };
    add(norm);
    add(norm.toUpper());
    add(norm.toLower());
    return candidates;
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

    QStringList names;
    names << "War3.mpq" << "War3x.mpq";

    QDir root(rootPath);
    const QStringList locals = root.entryList(QStringList() << "War3xLocal*.mpq",
                                              QDir::Files,
                                              QDir::Name);
    for (const auto& local : locals)
    {
        if (!names.contains(local))
            names << local;
    }

    // Patch goes last for highest priority.
    if (!names.contains("War3Patch.mpq"))
        names << "War3Patch.mpq";

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
            const DWORD err = GetLastError();
            LogSink::instance().log(QString("MPQ mount failed: %1 (err=%2)").arg(full).arg(err));
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

bool MpqVfs::openFileFromArchives(const QStringList& candidates, void** outFileHandle, QString* outArchive) const
{
    for (auto it = archives_.rbegin(); it != archives_.rend(); ++it)
    {
        for (const auto& candidate : candidates)
        {
            const QByteArray bytes = candidate.toUtf8();
            HANDLE hFile = nullptr;
            if (SFileOpenFileEx(it->handle, bytes.constData(), SFILE_OPEN_FROM_MPQ, &hFile))
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
    const QStringList candidates = buildCandidatePaths(path);

    void* hFile = nullptr;
    QString archive;
    const bool ok = openFileFromArchives(candidates, &hFile, &archive);
    if (ok && hFile)
        SFileCloseFile(hFile);
    return ok;
}

QByteArray MpqVfs::readAll(const QString& path) const
{
    const QStringList candidates = buildCandidatePaths(path);

    HANDLE hFile = nullptr;
    QString archive;
    if (!openFileFromArchives(candidates, (void**)&hFile, &archive))
    {
        LogSink::instance().log(QString("MPQ miss: %1").arg(path));
        if (!archives_.empty())
        {
            QStringList mounts;
            for (const auto& a : archives_)
                mounts << a.path;
            LogSink::instance().log(QString("MPQ mounted list: %1").arg(mounts.join("; ")));
        }
        if (!candidates.isEmpty())
            LogSink::instance().log(QString("MPQ tried: %1").arg(candidates.join(" | ")));
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
    const QStringList candidates = buildCandidatePaths(path);
    HANDLE hFile = nullptr;
    QString archive;
    if (openFileFromArchives(candidates, (void**)&hFile, &archive))
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
