#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QElapsedTimer>
#include "plotwidget.h"
#include "plotmatrix.h"
#include "filterablelistwidget.h"
#include "../plugins/dataloader_base.h"
#include "../plugins/statepublisher_base.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void undoableChangeHappened();

private slots:

    void onTrackerTimeUpdated(double current_time );

    void on_splitter_splitterMoved(int, int);

    void resizeEvent(QResizeEvent *) ;

    void on_pushAddRow_pressed();

    void on_pushAddColumn_pressed();

    void createActions();

    void on_pushremoveEmpty_pressed();

    void on_plotAdded(PlotWidget* widget);

    void on_addTabButton_pressed();

    void onActionSaveLayout();

    void onActionLoadLayout(bool reload_previous = false);

    void onActionLoadDataFile(bool reload_previous = false);

    void onActionReloadDataFile();

    void onActionReloadLayout();

    void on_pushHorizontalResize_pressed();

    void on_pushVerticalResize_pressed();

    void on_pushLinkHorizontalScale_toggled(bool checked);

    void on_pushButtonActivateTracker_toggled(bool checked);

    void on_pushButtonUndo_clicked(bool checked);

    void on_pushButtonRedo_clicked(bool checked);

    void on_tabWidget_currentChanged(int index);

    void on_tabWidget_tabCloseRequested(int index);

    void on_horizontalSlider_sliderMoved(int position);

private:
    Ui::MainWindow *ui;

    QAction* loadRecentFile;
    QAction* loadRecentLayout;

    QAction* actionSave_layout;
    QAction* actionLoad_layout;
    QAction* actionLoadData;

    QAction* actionUndo;
    QAction* actionRedo;

    FilterableListWidget* curvelist_widget;

    PlotMatrix *currentPlotGrid();

    void buildData();

    PlotDataMap    _mapped_plot_data;

    void rearrangeGridLayout();

    bool _horizontal_link;

    QColor colorHint();

    void loadPlugins(QString subdir_name);

    std::map<QString,DataLoader*> data_loader;
    std::vector<StatePublisher*>  state_publisher;

    QDomDocument xmlSaveState();
    void xmlLoadState(QDomDocument state_document);

    std::deque<QDomDocument> _undo_states;
    std::deque<QDomDocument> _redo_states;

    QElapsedTimer _undo_timer;


protected:
    void mousePressEvent(QMouseEvent *event) ;

    void dragEnterEvent(QDragEnterEvent *event) ;
    void dragMoveEvent(QDragMoveEvent *event) ;
    void dropEvent(QDropEvent *event) ;

    bool eventFilter(QObject *obj, QEvent *event);



    void deleteLoadedData();


};

#endif // MAINWINDOW_H
