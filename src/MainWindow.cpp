#include "MainWindow.h"

#include <QApplication>
#include <QFileDialog>
#include <QFileInfo>
#include <QDir>
#include <QDirIterator>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSplitter>
#include <QListView>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include <QMessageBox>
#include <QItemSelectionModel>
#include <QtConcurrent/QtConcurrent>

#include "GLModelView.h"
#include "MdxLoader.h"

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
}

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    buildUi();

    connect(&scanWatcher_, &QFutureWatcher<QStringList>::finished,
            this, &MainWindow::onFolderScanFinished);

    // Default: prompt user to pick a folder.
    lblStatus_->setText("Choose a folder containing .mdx files.");
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
    lblFolder_ = new QLabel("<no folder>");
    lblFolder_->setTextInteractionFlags(Qt::TextSelectableByMouse);

    top->addWidget(btnFolder_, 0);
    top->addWidget(lblFolder_, 1);
    root->addLayout(top);

    // Split view: left list + right viewer
    auto* splitter = new QSplitter(Qt::Horizontal);

    auto* left = new QWidget();
    auto* leftLayout = new QVBoxLayout(left);
    leftLayout->setContentsMargins(0, 0, 0, 0);
    leftLayout->setSpacing(6);

    editFilter_ = new QLineEdit();
    editFilter_->setPlaceholderText("Filter... (type to search)");
    leftLayout->addWidget(editFilter_);

    list_ = new QListView();
    list_->setUniformItemSizes(true);
    list_->setSelectionMode(QAbstractItemView::SingleSelection);
    leftLayout->addWidget(list_, 1);

    splitter->addWidget(left);

    viewer_ = new GLModelView();
    splitter->addWidget(viewer_);

    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);
    splitter->setSizes({ 320, 900 });

    root->addWidget(splitter, 1);

    // Status bar (simple label)
    lblStatus_ = new QLabel();
    lblStatus_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    root->addWidget(lblStatus_);

    // Models
    listModel_ = new QStringListModel(this);
    proxyModel_ = new QSortFilterProxyModel(this);
    proxyModel_->setSourceModel(listModel_);
    proxyModel_->setFilterCaseSensitivity(Qt::CaseInsensitive);
    proxyModel_->setSortCaseSensitivity(Qt::CaseInsensitive);
    proxyModel_->setDynamicSortFilter(true);
    proxyModel_->sort(0);

    list_->setModel(proxyModel_);

    connect(btnFolder_, &QPushButton::clicked, this, &MainWindow::chooseFolder);
    connect(editFilter_, &QLineEdit::textChanged, this, &MainWindow::onFilterTextChanged);

    connect(list_->selectionModel(), &QItemSelectionModel::currentChanged,
            this, &MainWindow::onSelectionChanged);

    connect(viewer_, &GLModelView::statusTextChanged, this, [this](const QString& t){
        lblStatus_->setText(t);
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
    lblFolder_->setText(folder);
    lblStatus_->setText("Scanning for .mdx files...");

    // Clear UI
    files_.clear();
    listModel_->setStringList({});
    viewer_->setModel(std::nullopt, "No model loaded");

    // Run scan in background
    scanWatcher_.setFuture(QtConcurrent::run(ScanMdxFiles, folder));
}

void MainWindow::onFolderScanFinished()
{
    files_ = scanWatcher_.result();

    QStringList display;
    display.reserve(files_.size());
    for (const auto& p : files_)
        display << DisplayNameFromPath(currentFolder_, p);

    listModel_->setStringList(display);
    lblStatus_->setText(QString("Found %1 .mdx files. Select one to preview.").arg(files_.size()));

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

void MainWindow::onSelectionChanged(const QModelIndex& current, const QModelIndex& /*previous*/)
{
    if (!current.isValid()) return;

    // Map to source row, then to full path
    const QModelIndex src = proxyModel_->mapToSource(current);
    const int row = src.row();
    if (row < 0 || row >= files_.size()) return;

    loadSelectedModel(files_[row]);
}

void MainWindow::loadSelectedModel(const QString& filePath)
{
    QString err;
    auto model = MdxLoader::LoadFromFile(filePath, &err);
    if (!model)
    {
        viewer_->setModel(std::nullopt, QFileInfo(filePath).fileName());
        lblStatus_->setText(QString("%1 | load failed: %2").arg(QFileInfo(filePath).fileName()).arg(err));
        return;
    }

    viewer_->setModel(model, QFileInfo(filePath).fileName());
}
