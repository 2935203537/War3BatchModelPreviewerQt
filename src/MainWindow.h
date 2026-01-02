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
class QDockWidget;
class QPlainTextEdit;
class QTabWidget;
class GLModelView;
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
    QLineEdit* editFilter_ = nullptr;
    QListView* list_ = nullptr;
    QListView* grid_ = nullptr;
    QLabel* lblFolder_ = nullptr;
    QLabel* statusLabel_ = nullptr;
    QLabel* speedLabel_ = nullptr;
    QSlider* speedSlider_ = nullptr;
    GLModelView* viewer_ = nullptr;
    QDockWidget* logDock_ = nullptr;
    QPlainTextEdit* logView_ = nullptr;
    QTabWidget* viewTabs_ = nullptr;

    // Models
    QStandardItemModel* listModel_ = nullptr;
    QSortFilterProxyModel* proxyModel_ = nullptr;
    QHash<QString, std::shared_ptr<ModelData>> modelCache_;

    // Background scan
    QFutureWatcher<QStringList> scanWatcher_;
    QFutureWatcher<struct ModelLoadResult> modelWatcher_;
    int loadToken_ = 0;
};
