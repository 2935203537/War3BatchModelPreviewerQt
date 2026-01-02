#include "MainWindow.h"

#include <QApplication>
#include <QFileDialog>
#include <QFileInfo>
#include <QFile>
#include <QDir>
#include <QDirIterator>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSplitter>
#include <QListView>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include <QDockWidget>
#include <QPlainTextEdit>
#include <QTabWidget>
#include <QPainter>
#include <QStandardItem>
#include <QMessageBox>
#include <QItemSelectionModel>
#include <QStatusBar>
#include <QSlider>
#include <QProcess>
#include <QDateTime>
#include <QtConcurrent/QtConcurrent>

#include "GLModelView.h"
#include "MdxLoader.h"
#include "LogSink.h"
#include "Vfs.h"

namespace
{
    static QStringList ScanMdxFiles(const QString& folder)
    {
        QStringList out;
        QDirIterator it(folder, QStringList() << "*.mdx" << "*.MDX",
                        QDir::Files, QDirIterator::Subdirectories);
        while (it.hasNext())
            out << it.next();
        out.sort(Qt::CaseInsensitive);
        return out;
    }

    static QString DisplayNameFromPath(const QString& baseFolder, const QString& path)
    {
        QDir base(baseFolder);
        const QString rel = base.relativeFilePath(path);
        return rel.isEmpty() ? QFileInfo(path).fileName() : rel;
    }

    static QIcon MakePlaceholderThumb(const QString& text, int size)
    {
        QPixmap pix(size, size);
        pix.fill(QColor(32, 32, 36));

        QPainter p(&pix);
        p.setRenderHint(QPainter::Antialiasing, true);
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(55, 55, 62));
        p.drawRoundedRect(QRectF(6, 6, size - 12, size - 12), 10, 10);

        QFont f = p.font();
        f.setPointSize(std::max(8, size / 10));
        f.setBold(true);
        p.setFont(f);
        p.setPen(QColor(220, 220, 220));
        p.drawText(pix.rect().adjusted(10, 10, -10, -10), Qt::AlignCenter | Qt::TextWordWrap, text);
        return QIcon(pix);
    }

    static ModelLoadResult LoadModelFile(const QString& filePath, int token)
    {
        ModelLoadResult result;
        result.path = filePath;
        result.token = token;
        QString err;
        result.model = MdxLoader::LoadFromFile(filePath, &err);
        result.error = err;
        return result;
    }
}

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    buildUi();

    connect(&scanWatcher_, &QFutureWatcher<QStringList>::finished,
            this, &MainWindow::onFolderScanFinished);
    connect(&modelWatcher_, &QFutureWatcher<ModelLoadResult>::finished,
            this, &MainWindow::onModelLoadFinished);

    diskVfs_ = std::make_shared<DiskVfs>(QString());
    mpqVfs_ = std::make_shared<MpqVfs>();
    vfs_ = std::make_shared<CompositeVfs>();
    vfs_->add(diskVfs_);
    vfs_->add(mpqVfs_);
    if (viewer_)
        viewer_->setVfs(vfs_);

    onWar3RootChanged();

    // Default: prompt user to pick a folder.
    statusLabel_->setText("Choose a folder containing .mdx files.");
}

MainWindow::~MainWindow()
{
    scanWatcher_.cancel();
    scanWatcher_.waitForFinished();
}

void MainWindow::buildUi()
{
    setWindowTitle("War3 Batch Model Previewer (Qt)");

    central_ = new QWidget(this);
    setCentralWidget(central_);

    auto* root = new QVBoxLayout(central_);
    root->setContentsMargins(8, 8, 8, 8);
    root->setSpacing(8);

    // Top bar
    auto* top = new QHBoxLayout();
    btnFolder_ = new QPushButton("Open Folder...");
    btnResetView_ = new QPushButton("Reset View");
    btnExportDiag_ = new QPushButton("Export Diagnostics...");
    lblFolder_ = new QLabel("<no folder>");
    lblFolder_->setTextInteractionFlags(Qt::TextSelectableByMouse);

    top->addWidget(btnFolder_, 0);
    top->addWidget(btnResetView_, 0);
    top->addWidget(btnExportDiag_, 0);
    top->addWidget(lblFolder_, 1);
    root->addLayout(top);

    // War3 root path
    auto* war3Row = new QHBoxLayout();
    lblWar3Root_ = new QLabel("War3 Root:");
    editWar3Root_ = new QLineEdit("E:\\Warcraft III Frozen Throne");
    btnWar3Browse_ = new QPushButton("Browse...");
    war3Row->addWidget(lblWar3Root_, 0);
    war3Row->addWidget(editWar3Root_, 1);
    war3Row->addWidget(btnWar3Browse_, 0);
    root->addLayout(war3Row);

    // Split view: left list + right viewer
    auto* splitter = new QSplitter(Qt::Horizontal);

    auto* left = new QWidget();
    auto* leftLayout = new QVBoxLayout(left);
    leftLayout->setContentsMargins(0, 0, 0, 0);
    leftLayout->setSpacing(6);

    editFilter_ = new QLineEdit();
    editFilter_->setPlaceholderText("Filter... (type to search)");
    leftLayout->addWidget(editFilter_);

    viewTabs_ = new QTabWidget();
    viewTabs_->setTabPosition(QTabWidget::North);

    list_ = new QListView();
    list_->setUniformItemSizes(true);
    list_->setSelectionMode(QAbstractItemView::SingleSelection);
    list_->setAlternatingRowColors(true);

    grid_ = new QListView();
    grid_->setViewMode(QListView::IconMode);
    grid_->setUniformItemSizes(true);
    grid_->setSelectionMode(QAbstractItemView::SingleSelection);
    grid_->setResizeMode(QListView::Adjust);
    grid_->setSpacing(8);
    grid_->setIconSize(QSize(128, 128));
    grid_->setGridSize(QSize(150, 150));

    viewTabs_->addTab(list_, "List");
    viewTabs_->addTab(grid_, "Grid");
    leftLayout->addWidget(viewTabs_, 1);

    splitter->addWidget(left);

    viewer_ = new GLModelView();
    splitter->addWidget(viewer_);

    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);
    splitter->setSizes({ 320, 900 });

    root->addWidget(splitter, 1);

    // Status bar (QMainWindow)
    statusLabel_ = new QLabel();
    statusLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    statusLabel_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    speedLabel_ = new QLabel("1.00x");
    speedLabel_->setMinimumWidth(52);
    speedLabel_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    speedSlider_ = new QSlider(Qt::Horizontal);
    speedSlider_->setRange(10, 300); // 0.10x .. 3.00x
    speedSlider_->setValue(100);     // 1.00x default
    speedSlider_->setFixedWidth(160);
    speedSlider_->setToolTip("Playback speed");

    statusBar()->addWidget(statusLabel_, 1);
    mpqStatusLabel_ = new QLabel("MPQ mounted: 0");
    statusBar()->addPermanentWidget(mpqStatusLabel_);
    statusBar()->addPermanentWidget(speedLabel_);
    statusBar()->addPermanentWidget(speedSlider_);

    // Log dock
    logDock_ = new QDockWidget("Log", this);
    logDock_->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);
    logView_ = new QPlainTextEdit();
    logView_->setReadOnly(true);
    logView_->setLineWrapMode(QPlainTextEdit::NoWrap);
    logDock_->setWidget(logView_);
    addDockWidget(Qt::BottomDockWidgetArea, logDock_);

    // Missing textures dock
    missingDock_ = new QDockWidget("Missing Textures", this);
    missingDock_->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);
    missingView_ = new QPlainTextEdit();
    missingView_->setReadOnly(true);
    missingView_->setLineWrapMode(QPlainTextEdit::NoWrap);
    missingDock_->setWidget(missingView_);
    addDockWidget(Qt::BottomDockWidgetArea, missingDock_);

    // Models
    listModel_ = new QStandardItemModel(this);
    proxyModel_ = new QSortFilterProxyModel(this);
    proxyModel_->setSourceModel(listModel_);
    proxyModel_->setFilterCaseSensitivity(Qt::CaseInsensitive);
    proxyModel_->setSortCaseSensitivity(Qt::CaseInsensitive);
    proxyModel_->setDynamicSortFilter(true);
    proxyModel_->sort(0);

    list_->setModel(proxyModel_);
    grid_->setModel(proxyModel_);
    grid_->setSelectionModel(list_->selectionModel());

    connect(btnFolder_, &QPushButton::clicked, this, &MainWindow::chooseFolder);
    connect(btnResetView_, &QPushButton::clicked, this, [this](){
        if (viewer_)
            viewer_->resetView();
    });
    connect(btnExportDiag_, &QPushButton::clicked, this, &MainWindow::exportDiagnostics);
    connect(btnWar3Browse_, &QPushButton::clicked, this, [this](){
        const QString folder = QFileDialog::getExistingDirectory(this, "Choose Warcraft III Root", editWar3Root_->text());
        if (!folder.isEmpty())
            editWar3Root_->setText(folder);
    });
    connect(editWar3Root_, &QLineEdit::editingFinished, this, &MainWindow::onWar3RootChanged);
    connect(editFilter_, &QLineEdit::textChanged, this, &MainWindow::onFilterTextChanged);

    connect(list_->selectionModel(), &QItemSelectionModel::currentChanged,
            this, &MainWindow::onSelectionChanged);

    connect(viewer_, &GLModelView::statusTextChanged, this, [this](const QString& t){
        statusLabel_->setText(t);
    });
    connect(viewer_, &GLModelView::missingTexturesChanged, this, [this](const QStringList& list){
        if (!missingView_)
            return;
        missingView_->setPlainText(list.join("\n\n"));
    });
    connect(&LogSink::instance(), &LogSink::messageAdded, this, [this](const QString& line){
        if (logView_)
            logView_->appendPlainText(line);
    });

    connect(speedSlider_, &QSlider::valueChanged, this, [this](int v){
        const float s = float(v) / 100.0f;
        speedLabel_->setText(QString::asprintf("%.2fx", double(s)));
        if (viewer_)
            viewer_->setPlaybackSpeed(s);
    });

    resize(1280, 720);
}

void MainWindow::chooseFolder()
{
    const QString folder = QFileDialog::getExistingDirectory(this, "Choose a folder with MDX files", currentFolder_);
    if (folder.isEmpty()) return;
    startScanFolder(folder);
}

void MainWindow::startScanFolder(const QString& folder)
{
    currentFolder_ = folder;
    viewer_->setAssetRoot(currentFolder_);
    if (diskVfs_)
        diskVfs_->setRootPath(currentFolder_);
    lblFolder_->setText(folder);
    statusLabel_->setText("Scanning for .mdx files...");

    // Clear UI
    files_.clear();
    listModel_->clear();
    viewer_->setModel(std::nullopt, "No model loaded", QString());

    // Run scan in background
    scanWatcher_.setFuture(QtConcurrent::run(ScanMdxFiles, folder));
}

void MainWindow::onFolderScanFinished()
{
    files_ = scanWatcher_.result();

    listModel_->clear();
    listModel_->setColumnCount(1);
    for (const auto& p : files_)
    {
        const QString display = DisplayNameFromPath(currentFolder_, p);
        auto* item = new QStandardItem(display);
        item->setData(p, Qt::UserRole);
        item->setEditable(false);
        item->setIcon(MakePlaceholderThumb(display, 128));
        listModel_->appendRow(item);
    }
    statusLabel_->setText(QString("Found %1 .mdx files. Select one to preview.").arg(files_.size()));

    // Select the first item automatically
    if (!files_.isEmpty())
    {
        const QModelIndex first = proxyModel_->index(0, 0);
        list_->setCurrentIndex(first);
    }
}

void MainWindow::onFilterTextChanged(const QString& text)
{
    proxyModel_->setFilterFixedString(text);

    // Auto-select first match after filtering
    if (proxyModel_->rowCount() > 0)
    {
        list_->setCurrentIndex(proxyModel_->index(0, 0));
    }
}

void MainWindow::onModelLoadFinished()
{
    const ModelLoadResult result = modelWatcher_.result();
    if (result.token != loadToken_)
        return;

    if (!result.model)
    {
        viewer_->setModel(std::nullopt, QFileInfo(result.path).fileName(), result.path);
        statusLabel_->setText(QString("%1 | load failed: %2")
                                  .arg(QFileInfo(result.path).fileName())
                                  .arg(result.error));
        LogSink::instance().log(QString("Load failed: %1 | %2").arg(result.path, result.error));
        return;
    }

    auto shared = std::make_shared<ModelData>(std::move(*result.model));
    modelCache_.insert(result.path, shared);

    viewer_->setModel(std::optional<ModelData>(*shared), QFileInfo(result.path).fileName(), result.path);
    LogSink::instance().log(QString("Loaded model: %1 | verts %2 | tris %3")
                                .arg(result.path)
                                .arg(shared->vertices.size())
                                .arg(shared->indices.size() / 3));
}

void MainWindow::onWar3RootChanged()
{
    const QString root = editWar3Root_ ? editWar3Root_->text().trimmed() : QString();
    if (!mpqVfs_)
        return;

    if (!QDir(root).exists())
    {
        if (mpqStatusLabel_)
            mpqStatusLabel_->setText("MPQ mounted: 0");
        LogSink::instance().log(QString("War3 root not found: %1").arg(root));
        return;
    }

    const bool mounted = mpqVfs_->mountWar3Root(root);
    const int count = mpqVfs_->mountedCount();
    if (mpqStatusLabel_)
        mpqStatusLabel_->setText(QString("MPQ mounted: %1").arg(count));
    if (!mounted)
        LogSink::instance().log(QString("No MPQ archives mounted from: %1").arg(root));
}

void MainWindow::exportDiagnostics()
{
    const QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    const QString defaultZip = QDir(QDir::current()).filePath(QString("logs/diagnostics_%1.zip").arg(timestamp));
    const QString zipPath = QFileDialog::getSaveFileName(this, "Export Diagnostics", defaultZip, "Zip (*.zip)");
    if (zipPath.isEmpty())
        return;

    const QString stagingRoot = QDir(QDir::current()).filePath("logs/diagnostic_tmp");
    QDir stagingDir(stagingRoot);
    if (stagingDir.exists())
        stagingDir.removeRecursively();
    QDir().mkpath(stagingRoot);

    auto copyFile = [&](const QString& src, const QString& relDst)
    {
        const QString dst = QDir(stagingRoot).filePath(relDst);
        QDir().mkpath(QFileInfo(dst).absolutePath());
        QFile::remove(dst);
        if (QFileInfo::exists(src))
            QFile::copy(src, dst);
    };

    copyFile(QDir(QDir::current()).filePath("logs/latest.log"), "logs/latest.log");
    copyFile(QDir(QDir::current()).filePath("out/mdx_debug.log"), "logs/mdx_debug.log");
    copyFile(QDir(QDir::current()).filePath("README.md"), "README.md");
    copyFile(QDir(QDir::current()).filePath("CMakeLists.txt"), "CMakeLists.txt");
    copyFile(QDir(QDir::current()).filePath("CMakePresets.json"), "CMakePresets.json");
    copyFile(QDir(QDir::current()).filePath("src/GLModelView.cpp"), "src/GLModelView.cpp");
    copyFile(QDir(QDir::current()).filePath("src/GLModelView.h"), "src/GLModelView.h");
    copyFile(QDir(QDir::current()).filePath("src/MdxLoader.cpp"), "src/MdxLoader.cpp");
    copyFile(QDir(QDir::current()).filePath("src/MdxLoader.h"), "src/MdxLoader.h");
    copyFile(QDir(QDir::current()).filePath("src/BlpLoader.cpp"), "src/BlpLoader.cpp");
    copyFile(QDir(QDir::current()).filePath("src/BlpLoader.h"), "src/BlpLoader.h");
    copyFile(QDir(QDir::current()).filePath("src/ModelData.h"), "src/ModelData.h");

    const QString cmd = QString("Compress-Archive -Force -Path \"%1\\*\" -DestinationPath \"%2\"")
                            .arg(stagingRoot)
                            .arg(zipPath);
    const int code = QProcess::execute("powershell", {"-NoProfile", "-Command", cmd});
    if (code != 0)
    {
        QMessageBox::warning(this, "Export Diagnostics", "Failed to create diagnostics zip.");
        LogSink::instance().log(QString("Diagnostics export failed: %1").arg(zipPath));
        return;
    }

    LogSink::instance().log(QString("Diagnostics exported: %1").arg(zipPath));
    QMessageBox::information(this, "Export Diagnostics", "Diagnostics package created.");
}

void MainWindow::onSelectionChanged(const QModelIndex& current, const QModelIndex& /*previous*/)
{
    if (!current.isValid()) return;

    // Map to source row, then to full path
    const QModelIndex src = proxyModel_->mapToSource(current);
    const QString filePath = src.data(Qt::UserRole).toString();
    if (filePath.isEmpty()) return;

    loadSelectedModel(filePath);
}

void MainWindow::loadSelectedModel(const QString& filePath)
{
    if (auto it = modelCache_.find(filePath); it != modelCache_.end())
    {
        if (it.value())
            viewer_->setModel(std::optional<ModelData>(*it.value()), QFileInfo(filePath).fileName(), filePath);
        else
            viewer_->setModel(std::nullopt, QFileInfo(filePath).fileName(), filePath);
        LogSink::instance().log(QString("Loaded model from cache: %1").arg(filePath));
        return;
    }

    const int token = ++loadToken_;
    statusLabel_->setText(QString("%1 | loading...").arg(QFileInfo(filePath).fileName()));
    LogSink::instance().log(QString("Loading model async: %1").arg(filePath));

    modelWatcher_.setFuture(QtConcurrent::run(LoadModelFile, filePath, token));
}
