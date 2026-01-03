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
#include <QComboBox>
#include <QGroupBox>
#include <QFrame>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QSignalBlocker>
#include <QTextStream>
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

    static QString filterModeName(std::uint32_t mode)
    {
        switch (mode)
        {
        case 0: return "None";
        case 1: return "Transparent";
        case 2: return "Blend";
        case 3: return "Additive";
        case 4: return "AddAlpha";
        case 5: return "Modulate";
        case 6: return "Modulate2x";
        default: return "Blend";
        }
    }

    static void writeMdxAsMdl(const ModelData& model, const QString& path, const QString& name)
    {
        QFile f(path);
        if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
            return;

        QTextStream ts(&f);
        const int formatVersion = (model.mdxVersion >= 1000) ? 1000 : 800;
        ts << "Version {\n\tFormatVersion " << formatVersion << ",\n}\n";
        ts << "Model \"" << name << "\" {\n";
        ts << "\tNumGeosets " << model.geosetDiagnostics.size() << ",\n";
        ts << "\tNumGeosetAnims 0,\n";

        int bones = 0, helpers = 0, attachments = 0;
        for (const auto& n : model.nodes)
        {
            if (n.type == "BONE")
                bones++;
            else if (n.type == "HELP")
                helpers++;
            else if (n.type == "ATCH")
                attachments++;
            else
                helpers++;
        }
        if (bones > 0) ts << "\tNumBones " << bones << ",\n";
        if (helpers > 0) ts << "\tNumHelpers " << helpers << ",\n";
        if (attachments > 0) ts << "\tNumAttachments " << attachments << ",\n";

        if (model.hasBounds)
        {
            ts << "\tMinimumExtent { " << model.boundsMin[0] << ", " << model.boundsMin[1] << ", " << model.boundsMin[2] << " },\n";
            ts << "\tMaximumExtent { " << model.boundsMax[0] << ", " << model.boundsMax[1] << ", " << model.boundsMax[2] << " },\n";
        }
        ts << "}\n";

        if (!model.sequences.empty())
        {
            ts << "Sequences " << model.sequences.size() << " {\n";
            for (const auto& s : model.sequences)
            {
                ts << "\tAnim \"" << QString::fromStdString(s.name) << "\" {\n";
                ts << "\t\tInterval { " << s.startMs << ", " << s.endMs << " },\n";
                if (s.flags & 1)
                    ts << "\t\tNonLooping,\n";
                ts << "\t}\n";
            }
            ts << "}\n";
        }

        if (!model.textures.empty())
        {
            ts << "Textures " << model.textures.size() << " {\n";
            for (const auto& t : model.textures)
            {
                ts << "\tBitmap {\n";
                if (!t.fileName.empty())
                    ts << "\t\tImage \"" << QString::fromStdString(t.fileName) << "\",\n";
                if (t.replaceableId != 0)
                    ts << "\t\tReplaceableId " << t.replaceableId << ",\n";
                ts << "\t\tWrapWidth,\n\t\tWrapHeight,\n";
                ts << "\t}\n";
            }
            ts << "}\n";
        }

        if (!model.materials.empty())
        {
            ts << "Materials " << model.materials.size() << " {\n";
            for (const auto& m : model.materials)
            {
                ts << "\tMaterial {\n";
                ts << "\t\tLayer {\n";
                ts << "\t\t\tFilterMode " << filterModeName(m.layer.filterMode) << ",\n";
                ts << "\t\t\tstatic TextureID " << m.layer.textureId << ",\n";
                ts << "\t\t\tAlpha " << m.layer.alpha << ",\n";
                ts << "\t\t}\n";
                ts << "\t}\n";
            }
            ts << "}\n";
        }

        for (std::size_t gi = 0; gi < model.geosetDiagnostics.size(); ++gi)
        {
            const auto& gd = model.geosetDiagnostics[gi];
            ts << "Geoset {\n";
            ts << "\tVertices " << gd.vertexCount << " {\n";
            for (std::uint32_t i = 0; i < gd.vertexCount; ++i)
            {
                const auto& v = model.vertices[gd.baseVertex + i];
                ts << "\t\t{ " << v.px << ", " << v.py << ", " << v.pz << " },\n";
            }
            ts << "\t}\n";
            ts << "\tNormals " << gd.vertexCount << " {\n";
            for (std::uint32_t i = 0; i < gd.vertexCount; ++i)
            {
                const auto& v = model.vertices[gd.baseVertex + i];
                ts << "\t\t{ " << v.nx << ", " << v.ny << ", " << v.nz << " },\n";
            }
            ts << "\t}\n";
            ts << "\tTVertices " << gd.vertexCount << " {\n";
            for (std::uint32_t i = 0; i < gd.vertexCount; ++i)
            {
                const auto& v = model.vertices[gd.baseVertex + i];
                ts << "\t\t{ " << v.u << ", " << v.v << " },\n";
            }
            ts << "\t}\n";

            ts << "\tVertexGroup {\n";
            for (std::uint8_t vg : gd.gndx)
                ts << "\t\t" << int(vg) << ",\n";
            ts << "\t}\n";

            ts << "\tFaces 1 " << gd.indexCount << " {\n\t\tTriangles {\n\t\t\t{ ";
            for (std::uint32_t i = 0; i < gd.indexCount; ++i)
            {
                const std::uint32_t idx = model.indices[gd.indexOffset + i] - gd.baseVertex;
                ts << idx;
                if (i + 1 < gd.indexCount)
                    ts << ", ";
            }
            ts << " },\n\t\t}\n\t}\n";

            ts << "\tGroups " << gd.mtgc.size() << " " << gd.mats.size() << " {\n";
            if (!gd.expandedGroups.empty())
            {
                for (const auto& group : gd.expandedGroups)
                {
                    ts << "\t\tMatrices { ";
                    for (int i = 0; i < int(group.size()); ++i)
                    {
                        ts << group[std::size_t(i)];
                        if (i + 1 < int(group.size()))
                            ts << ", ";
                    }
                    ts << " },\n";
                }
            }
            else
            {
                std::size_t offset = 0;
                for (std::uint32_t sz : gd.mtgc)
                {
                    ts << "\t\tMatrices { ";
                    for (std::uint32_t k = 0; k < sz && offset < gd.mats.size(); ++k, ++offset)
                    {
                        ts << gd.mats[offset];
                        if (k + 1 < sz && offset + 1 < gd.mats.size())
                            ts << ", ";
                    }
                    ts << " },\n";
                }
            }
            ts << "\t}\n";
            ts << "\tMaterialID " << gd.materialId << ",\n";
            ts << "\tSelectionGroup 0,\n";
            ts << "}\n";
        }

        if (!model.nodes.empty())
        {
            for (const auto& n : model.nodes)
            {
                QString type = QString::fromStdString(n.type);
                if (type.isEmpty())
                    type = "Helper";
                if (type == "BONE")
                    type = "Bone";
                else if (type == "HELP")
                    type = "Helper";
                else if (type == "ATCH")
                    type = "Attachment";
                else
                    type = "Helper";

                const QString nodeName = QString::fromStdString(n.name);
                ts << type << " \"" << nodeName << "\" {\n";
                ts << "\tObjectId " << n.nodeId << ",\n";
                if (n.parentId >= 0)
                    ts << "\tParent " << n.parentId << ",\n";
                ts << "\tPivotPoint { " << n.pivot.x << ", " << n.pivot.y << ", " << n.pivot.z << " },\n";
                if (type == "Bone")
                {
                    ts << "\tGeosetId -1,\n";
                    ts << "\tGeosetAnimId -1,\n";
                }
                if (type == "Attachment")
                {
                    ts << "\tPath \"\",\n";
                    ts << "\tAttachmentID 0,\n";
                }
                ts << "}\n";
            }
        }

        if (!model.pivots.empty())
        {
            ts << "PivotPoints " << model.pivots.size() << " {\n";
            for (const auto& p : model.pivots)
                ts << "\t{ " << p.x << ", " << p.y << ", " << p.z << " },\n";
            ts << "}\n";
        }
        else if (!model.nodes.empty())
        {
            ts << "PivotPoints " << model.nodes.size() << " {\n";
            for (const auto& n : model.nodes)
                ts << "\t{ " << n.pivot.x << ", " << n.pivot.y << ", " << n.pivot.z << " },\n";
            ts << "}\n";
        }
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

    // Main area: left list + viewer + right panel
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

    auto* viewerHost = new QWidget();
    viewerHost->setStyleSheet("background: #000;");
    auto* viewerHostLayout = new QVBoxLayout(viewerHost);
    viewerHostLayout->setContentsMargins(0, 0, 0, 0);
    viewerHostLayout->setSpacing(0);

    auto* viewerFrame = new QFrame();
    viewerFrame->setStyleSheet("QFrame { background: #000; border: none; }");
    viewerFrame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto* viewerFrameLayout = new QVBoxLayout(viewerFrame);
    viewerFrameLayout->setContentsMargins(0, 0, 0, 0);
    viewerFrameLayout->setSpacing(0);

    viewer_ = new GLModelView();
    viewer_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    viewerFrameLayout->addWidget(viewer_);

    viewerHostLayout->addWidget(viewerFrame, 1);
    splitter->addWidget(viewerHost);

    auto* panel = new QFrame();
    panel->setStyleSheet(
        "QFrame { background: #f7f7f9; border-radius: 8px; }"
        "QGroupBox { font-weight: 600; border: none; margin-top: 8px; }"
        "QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 2px 0; }"
        "QPushButton { background: #2f2f33; color: #f2f2f2; border-radius: 6px; padding: 6px 10px; }"
        "QPushButton:hover { background: #3a3a40; }"
        "QComboBox, QLineEdit { background: #2f2f33; color: #f2f2f2; border-radius: 6px; padding: 4px 8px; }"
        "QSlider::groove:horizontal { height: 6px; background: #dcdde1; border-radius: 3px; }"
        "QSlider::handle:horizontal { width: 14px; margin: -4px 0; background: #5b8cff; border-radius: 7px; }"
    );
    panel->setFixedWidth(320);

    auto* panelLayout = new QVBoxLayout(panel);
    panelLayout->setContentsMargins(16, 16, 16, 16);
    panelLayout->setSpacing(14);

    lblModelName_ = new QLabel("No model loaded");
    lblModelName_->setStyleSheet("font-weight: 600;");
    lblModelName_->setWordWrap(true);
    panelLayout->addWidget(lblModelName_);

    auto* controlBox = new QGroupBox("Controls");
    auto* controlLayout = new QVBoxLayout(controlBox);
    controlLayout->setSpacing(8);

    auto* animLabel = new QLabel("Animation");
    auto* animCombo = new QComboBox();
    animCombo->addItem("[0]Stand");
    controlLayout->addWidget(animLabel);
    controlLayout->addWidget(animCombo);

    auto* teamLabel = new QLabel("Team Color");
    auto* teamCombo = new QComboBox();
    teamCombo->addItems({"[0]Red", "[1]Blue", "[2]Teal", "[3]Purple"});
    controlLayout->addWidget(teamLabel);
    controlLayout->addWidget(teamCombo);

    auto* speedTitle = new QLabel("Playback Speed");
    speedSlider_ = new QSlider(Qt::Horizontal);
    speedSlider_->setRange(10, 300);
    speedSlider_->setValue(100);
    speedSlider_->setToolTip("Playback speed");
    controlLayout->addWidget(speedTitle);
    controlLayout->addWidget(speedSlider_);

    auto* bgTitle = new QLabel("Background Alpha");
    bgAlphaSlider_ = new QSlider(Qt::Horizontal);
    bgAlphaSlider_->setRange(0, 100);
    bgAlphaSlider_->setValue(100);
    controlLayout->addWidget(bgTitle);
    controlLayout->addWidget(bgAlphaSlider_);

    panelLayout->addWidget(controlBox);

    auto* cameraBox = new QGroupBox("Camera");
    auto* cameraLayout = new QFormLayout(cameraBox);
    cameraLayout->setLabelAlignment(Qt::AlignLeft);
    cameraLayout->setFormAlignment(Qt::AlignTop);
    cameraLayout->setHorizontalSpacing(10);
    cameraLayout->setVerticalSpacing(8);

    yawSpin_ = new QDoubleSpinBox();
    yawSpin_->setRange(-180.0, 180.0);
    yawSpin_->setDecimals(1);
    yawSpin_->setSingleStep(1.0);

    pitchSpin_ = new QDoubleSpinBox();
    pitchSpin_->setRange(-89.0, 89.0);
    pitchSpin_->setDecimals(1);
    pitchSpin_->setSingleStep(1.0);

    rollSpin_ = new QDoubleSpinBox();
    rollSpin_->setRange(-180.0, 180.0);
    rollSpin_->setDecimals(1);
    rollSpin_->setSingleStep(1.0);
    rollSpin_->setValue(0.0);

    yawSpin_->setValue(0.0);
    pitchSpin_->setValue(0.0);

    cameraLayout->addRow("Yaw", yawSpin_);
    cameraLayout->addRow("Pitch", pitchSpin_);
    cameraLayout->addRow("Roll", rollSpin_);

    panXSpin_ = new QDoubleSpinBox();
    panXSpin_->setRange(-99999.0, 99999.0);
    panXSpin_->setDecimals(2);
    panXSpin_->setSingleStep(1.0);

    panYSpin_ = new QDoubleSpinBox();
    panYSpin_->setRange(-99999.0, 99999.0);
    panYSpin_->setDecimals(2);
    panYSpin_->setSingleStep(1.0);

    panZSpin_ = new QDoubleSpinBox();
    panZSpin_->setRange(-99999.0, 99999.0);
    panZSpin_->setDecimals(2);
    panZSpin_->setSingleStep(1.0);

    cameraLayout->addRow("Pan X", panXSpin_);
    cameraLayout->addRow("Pan Y", panYSpin_);
    cameraLayout->addRow("Pan Z", panZSpin_);

    panelLayout->addWidget(cameraBox);

    auto* opsBox = new QGroupBox("Actions");
    auto* opsLayout = new QVBoxLayout(opsBox);
    opsLayout->setSpacing(8);

    opsLayout->addWidget(new QLabel("Export model and textures"));
    auto* exportDir = new QLineEdit();
    exportDir->setPlaceholderText("Choose folder...");
    opsLayout->addWidget(exportDir);
    auto* exportName = new QLineEdit();
    exportName->setPlaceholderText("Rename?");
    opsLayout->addWidget(exportName);
    auto* exportBtn = new QPushButton("Export");
    opsLayout->addWidget(exportBtn);

    panelLayout->addWidget(opsBox);
    panelLayout->addStretch(1);

    splitter->addWidget(panel);
    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);
    splitter->setStretchFactor(2, 0);
    splitter->setSizes({ 320, 900, 320 });

    root->addWidget(splitter, 1);
    // Status bar (QMainWindow)
    statusLabel_ = new QLabel();
    statusLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    statusLabel_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    statusBar()->addWidget(statusLabel_, 1);
    mpqStatusLabel_ = new QLabel("MPQ mounted: 0");
    statusBar()->addPermanentWidget(mpqStatusLabel_);

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
        if (viewer_)
            viewer_->setPlaybackSpeed(s);
    });
    connect(bgAlphaSlider_, &QSlider::valueChanged, this, [this](int v){
        const float a = float(v) / 100.0f;
        if (viewer_)
            viewer_->setBackgroundAlpha(a);
    });

    auto applyAngles = [this]() {
        if (!viewer_ || !yawSpin_ || !pitchSpin_ || !rollSpin_)
            return;
        viewer_->setCameraAngles(float(yawSpin_->value()),
                                 float(pitchSpin_->value()),
                                 float(rollSpin_->value()));
    };
    connect(yawSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [applyAngles](double){ applyAngles(); });
    connect(pitchSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [applyAngles](double){ applyAngles(); });
    connect(rollSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [applyAngles](double){ applyAngles(); });

    connect(viewer_, &GLModelView::anglesChanged, this, [this](float yaw, float pitch, float roll){
        if (!yawSpin_ || !pitchSpin_ || !rollSpin_)
            return;
        const QSignalBlocker blockYaw(*yawSpin_);
        const QSignalBlocker blockPitch(*pitchSpin_);
        const QSignalBlocker blockRoll(*rollSpin_);
        yawSpin_->setValue(yaw);
        pitchSpin_->setValue(pitch);
        rollSpin_->setValue(roll);
    });

    auto applyPan = [this]() {
        if (!viewer_ || !panXSpin_ || !panYSpin_ || !panZSpin_)
            return;
        viewer_->setCameraPan(float(panXSpin_->value()),
                              float(panYSpin_->value()),
                              float(panZSpin_->value()));
    };
    connect(panXSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [applyPan](double){ applyPan(); });
    connect(panYSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [applyPan](double){ applyPan(); });
    connect(panZSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [applyPan](double){ applyPan(); });

    connect(viewer_, &GLModelView::panChanged, this, [this](float x, float y, float z){
        if (!panXSpin_ || !panYSpin_ || !panZSpin_)
            return;
        const QSignalBlocker blockX(*panXSpin_);
        const QSignalBlocker blockY(*panYSpin_);
        const QSignalBlocker blockZ(*panZSpin_);
        panXSpin_->setValue(x);
        panYSpin_->setValue(y);
        panZSpin_->setValue(z);
    });

    resize(1280, 720);
}

void MainWindow::chooseFolder()
{
    const QString folder = QFileDialog::getExistingDirectory(this, "Choose a folder with MDX files", currentFolder_);
    if (folder.isEmpty())
        return;
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

    files_.clear();
    listModel_->clear();
    viewer_->setModel(std::nullopt, "No model loaded", QString());

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

    if (!files_.isEmpty())
    {
        const QModelIndex first = proxyModel_->index(0, 0);
        list_->setCurrentIndex(first);
    }
}

void MainWindow::onFilterTextChanged(const QString& text)
{
    proxyModel_->setFilterFixedString(text);

    if (proxyModel_->rowCount() > 0)
        list_->setCurrentIndex(proxyModel_->index(0, 0));
}

void MainWindow::onSelectionChanged(const QModelIndex& current, const QModelIndex& /*previous*/)
{
    if (!current.isValid())
        return;

    const QModelIndex src = proxyModel_->mapToSource(current);
    const QString filePath = src.data(Qt::UserRole).toString();
    if (filePath.isEmpty())
        return;

    loadSelectedModel(filePath);
}

void MainWindow::onModelLoadFinished()
{
    const ModelLoadResult result = modelWatcher_.result();
    if (result.token != loadToken_)
        return;

    const QString displayName = QFileInfo(result.path).fileName();
    if (lblModelName_)
        lblModelName_->setText(displayName);

    if (!result.model)
    {
        viewer_->setModel(std::nullopt, displayName, result.path);
        statusLabel_->setText(QString("%1 | load failed: %2")
                                  .arg(displayName)
                                  .arg(result.error));
        LogSink::instance().log(QString("Load failed: %1 | %2").arg(result.path, result.error));
        return;
    }

    auto shared = std::make_shared<ModelData>(std::move(*result.model));
    modelCache_.insert(result.path, shared);

    viewer_->setModel(std::optional<ModelData>(*shared), displayName, result.path);
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

    // Model diagnostics (geosets / groups / objects)
    {
        const QString diagDir = QDir(stagingRoot).filePath("diagnostics");
        QDir().mkpath(diagDir);
        QFile diagFile(QDir(diagDir).filePath("model_dump.txt"));
        if (diagFile.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
        {
            QTextStream ts(&diagFile);

            QString selectedPath;
            if (list_ && list_->selectionModel())
            {
                const QModelIndex current = list_->selectionModel()->currentIndex();
                if (current.isValid())
                {
                    const QModelIndex src = proxyModel_->mapToSource(current);
                    selectedPath = src.data(Qt::UserRole).toString();
                }
            }

            ts << "Selected model: " << (selectedPath.isEmpty() ? "<none>" : selectedPath) << "\n";

            std::shared_ptr<ModelData> model;
            if (!selectedPath.isEmpty())
            {
                if (auto it = modelCache_.find(selectedPath); it != modelCache_.end())
                {
                    model = it.value();
                }
                else
                {
                    QString err;
                    auto loaded = MdxLoader::LoadFromFile(selectedPath, &err);
                    if (loaded)
                        model = std::make_shared<ModelData>(std::move(*loaded));
                    else
                        ts << "Load failed: " << err << "\n";
                }
            }

            if (!model)
            {
                ts << "No model data available.\n";
            }
            else
            {
                ts << "Geosets: " << model->geosetDiagnostics.size() << "\n";

                QMap<int, int> groupSizeHist;
                QMap<int, int> vertexGroupUsage;

                for (std::size_t gi = 0; gi < model->geosetDiagnostics.size(); ++gi)
                {
                    const auto& gd = model->geosetDiagnostics[gi];
                    ts << "\n[Geoset " << gi << "]\n";
                    ts << "materialId: " << gd.materialId
                       << " | verts: " << gd.vertexCount
                       << " | tris: " << gd.triCount
                       << " | maxGNDX: " << gd.maxVertexGroup << "\n";
                    ts << "GNDX count: " << gd.gndx.size() << "\n";
                    ts << "MTGC count: " << gd.mtgc.size() << "\n";
                    ts << "MATS count: " << gd.mats.size() << "\n";

                    if (!gd.mtgc.empty())
                    {
                        for (std::uint8_t vg : gd.gndx)
                            Q_ASSERT(vg < gd.mtgc.size());
                    }

                    std::size_t mtgcSum = 0;
                    for (std::uint32_t sz : gd.mtgc)
                    {
                        mtgcSum += sz;
                        groupSizeHist[int(sz)] += 1;
                    }
                    Q_ASSERT(mtgcSum <= gd.mats.size());

                    auto joinU8 = [](const std::vector<std::uint8_t>& v) {
                        QStringList out;
                        out.reserve(int(v.size()));
                        for (auto x : v) out << QString::number(int(x));
                        return out.join(", ");
                    };
                    auto joinU32 = [](const std::vector<std::uint32_t>& v) {
                        QStringList out;
                        out.reserve(int(v.size()));
                        for (auto x : v) out << QString::number(x);
                        return out.join(", ");
                    };
                    auto joinI32 = [](const std::vector<std::int32_t>& v) {
                        QStringList out;
                        out.reserve(int(v.size()));
                        for (auto x : v) out << QString::number(x);
                        return out.join(", ");
                    };

                    ts << "GNDX: [" << joinU8(gd.gndx) << "]\n";
                    ts << "MTGC: [" << joinU32(gd.mtgc) << "]\n";
                    ts << "MATS: [" << joinI32(gd.mats) << "]\n";

                    ts << "Expanded groups:\n";
                    for (std::size_t g = 0; g < gd.expandedGroups.size(); ++g)
                        ts << "  [" << g << "] {" << joinI32(gd.expandedGroups[g]) << "}\n";
                }

                for (std::uint16_t gid : model->vertexGroups)
                {
                    Q_ASSERT(model->skinGroups.empty() || gid < model->skinGroups.size());
                    vertexGroupUsage[int(gid)] += 1;
                }

                ts << "\nGroup size histogram (MTGC sizes):\n";
                for (auto it = groupSizeHist.begin(); it != groupSizeHist.end(); ++it)
                    ts << "  size " << it.key() << ": " << it.value() << "\n";

                ts << "\nVertex group usage (GNDX -> group id):\n";
                for (auto it = vertexGroupUsage.begin(); it != vertexGroupUsage.end(); ++it)
                    ts << "  group " << it.key() << ": " << it.value() << "\n";

                ts << "\nobjectsById (type/name/parent/pivot):\n";
                for (std::size_t i = 0; i < model->nodes.size(); ++i)
                {
                    const auto& n = model->nodes[i];
                    const QString type = n.type.empty() ? "NODE" : QString::fromStdString(n.type);
                    const QString name = QString::fromStdString(n.name);
                    ts << "  [" << i << "] " << type << " | " << name
                       << " | parent=" << n.parentId
                       << " | pivot=(" << n.pivot.x << ", " << n.pivot.y << ", " << n.pivot.z << ")\n";
                }

                const QString mdlOut = QDir(diagDir).filePath(QString("%1_from_mdx.mdl")
                                                                  .arg(QFileInfo(selectedPath).completeBaseName()));
                writeMdxAsMdl(*model, mdlOut, QFileInfo(selectedPath).fileName());
            }
        }
    }

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

void MainWindow::loadSelectedModel(const QString& filePath)
{
    const QString displayName = QFileInfo(filePath).fileName();
    if (lblModelName_)
        lblModelName_->setText(displayName);

    if (auto it = modelCache_.find(filePath); it != modelCache_.end())
    {
        if (it.value())
            viewer_->setModel(std::optional<ModelData>(*it.value()), displayName, filePath);
        else
            viewer_->setModel(std::nullopt, displayName, filePath);
        LogSink::instance().log(QString("Loaded model from cache: %1").arg(filePath));
        return;
    }

    const int token = ++loadToken_;
    statusLabel_->setText(QString("%1 | loading...").arg(displayName));
    LogSink::instance().log(QString("Loading model async: %1").arg(filePath));

    modelWatcher_.setFuture(QtConcurrent::run(LoadModelFile, filePath, token));
}
