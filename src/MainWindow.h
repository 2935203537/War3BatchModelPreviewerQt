#pragma once
#include <QMainWindow>
#include <QFutureWatcher>
#include <QStandardItemModel>
#include <QModelIndex>
#include <QSortFilterProxyModel>
#include <optional>
#include <memory>
#include <QHash>

#include "ModelData.h"

class QListView;
class QLineEdit;
class QLabel;
class QPushButton;
class QSlider;
class QDoubleSpinBox;
class QDockWidget;
class QPlainTextEdit;
class QTabWidget;
class GLModelView;
class CompositeVfs;
class DiskVfs;
class MpqVfs;
struct ModelLoadResult
{
    QString path;
    std::optional<ModelData> model;
    QString error;
    int token = 0;
};

class MainWindow final : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

private slots:
    void chooseFolder();
    void onFolderScanFinished();
    void onSelectionChanged(const QModelIndex& current, const QModelIndex& previous);
    void onFilterTextChanged(const QString& text);
    void onModelLoadFinished();
    void exportDiagnostics();
    void onWar3RootChanged();

private:
    void buildUi();
    void startScanFolder(const QString& folder);
    void loadSelectedModel(const QString& filePath);

    QString currentFolder_;
    QStringList files_;

    // UI
    QWidget* central_ = nullptr;
    QPushButton* btnFolder_ = nullptr;
    QPushButton* btnResetView_ = nullptr;
    QPushButton* btnExportDiag_ = nullptr;
    QLabel* lblWar3Root_ = nullptr;
    QLineEdit* editWar3Root_ = nullptr;
    QPushButton* btnWar3Browse_ = nullptr;
    QLineEdit* editFilter_ = nullptr;
    QListView* list_ = nullptr;
    QListView* grid_ = nullptr;
    QLabel* lblFolder_ = nullptr;
    QLabel* lblModelName_ = nullptr;
    QLabel* statusLabel_ = nullptr;
    QSlider* speedSlider_ = nullptr;
    QSlider* bgAlphaSlider_ = nullptr;
    QDoubleSpinBox* yawSpin_ = nullptr;
    QDoubleSpinBox* pitchSpin_ = nullptr;
    QDoubleSpinBox* rollSpin_ = nullptr;
    QDoubleSpinBox* panXSpin_ = nullptr;
    QDoubleSpinBox* panYSpin_ = nullptr;
    QDoubleSpinBox* panZSpin_ = nullptr;
    QLabel* mpqStatusLabel_ = nullptr;
    GLModelView* viewer_ = nullptr;
    QDockWidget* logDock_ = nullptr;
    QPlainTextEdit* logView_ = nullptr;
    QDockWidget* missingDock_ = nullptr;
    QPlainTextEdit* missingView_ = nullptr;
    QTabWidget* viewTabs_ = nullptr;

    // Models
    QStandardItemModel* listModel_ = nullptr;
    QSortFilterProxyModel* proxyModel_ = nullptr;
    QHash<QString, std::shared_ptr<ModelData>> modelCache_;
    std::shared_ptr<CompositeVfs> vfs_;
    std::shared_ptr<DiskVfs> diskVfs_;
    std::shared_ptr<MpqVfs> mpqVfs_;

    QFutureWatcher<QStringList> scanWatcher_;
    QFutureWatcher<struct ModelLoadResult> modelWatcher_;
    int loadToken_ = 0;
};
