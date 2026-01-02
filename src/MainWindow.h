#pragma once
#include <QMainWindow>
#include <QFutureWatcher>
#include <QStringListModel>
#include <QModelIndex>
#include <QSortFilterProxyModel>
#include <optional>

#include "ModelData.h"

class QListView;
class QLineEdit;
class QLabel;
class QPushButton;
class QSlider;
class GLModelView;

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

private:
    void buildUi();
    void startScanFolder(const QString& folder);
    void loadSelectedModel(const QString& filePath);

    QString currentFolder_;
    QStringList files_;

    // UI
    QWidget* central_ = nullptr;
    QPushButton* btnFolder_ = nullptr;
    QLineEdit* editFilter_ = nullptr;
    QListView* list_ = nullptr;
    QLabel* lblFolder_ = nullptr;
    QLabel* statusLabel_ = nullptr;
    QLabel* speedLabel_ = nullptr;
    QSlider* speedSlider_ = nullptr;
    GLModelView* viewer_ = nullptr;

    // Models
    QStringListModel* listModel_ = nullptr;
    QSortFilterProxyModel* proxyModel_ = nullptr;

    // Background scan
    QFutureWatcher<QStringList> scanWatcher_;
};
