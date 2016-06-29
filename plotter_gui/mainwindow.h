#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QElapsedTimer>
#include "plotwidget.h"
#include "plotmatrix.h"
#include "filterablelistwidget.h"
#include "tabbedplotwidget.h"

#include "../plugins/dataloader_base.h"
#include "../plugins/statepublisher_base.h"
#include "../plugins/datastreamer_base.h"

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
    void onUndoableChange();

private slots:

    void onTrackerTimeUpdated(double current_time );

    void onTrackerPositionUpdated(QPointF pos );

    void onSplitterMoved(int, int);

    void resizeEvent(QResizeEvent *) ;

    void onPlotAdded(PlotWidget* plot);

    void onPlotMatrixAdded(PlotMatrix* matrix);

    void onTabAreaAdded(TabbedPlotWidget* tabbed_widget);

    void onActionSaveLayout();

    void onActionLoadLayout(bool reload_previous = false);

    void onActionLoadDataFile(bool reload_from_settings = false);

    void onActionLoadDataFileImpl(QString filename, bool reuse_last_timeindex = false );

    void onActionReloadDataFileFromSettings();

    void onActionReloadSameDataFile();

    void onActionReloadRecentLayout();

    void onActionLoadStreamer();

    void onUndoInvoked();

    void onRedoInvoked();

    void on_horizontalSlider_sliderMoved(int position);

    void on_tabbedAreaDestroyed(QObject*object);

    void onFloatingWindowDestroyed(QObject*object);

    void onCreateFloatingWindow(PlotMatrix* first_tab = NULL);

    void on_pushButtonAddSubwindow_pressed();

    void onSwapPlots(PlotWidget* source, PlotWidget* destination);

    void on_pushButtonStreaming_toggled(bool checked);

    void onReplotRequested();

    void on_streamingSpinBox_valueChanged(int value);

    void onDeleteLoadedData();

    void on_pushButtonActivateTracker_toggled(bool checked);

private:
    Ui::MainWindow *ui;

    std::vector<TabbedPlotWidget *> _tabbed_plotarea;
    std::vector<QMainWindow *>      _floating_window;

    QAction* _actionUndo;
    QAction* _actionRedo;

    void createActions();

    FilterableListWidget* curvelist_widget;

   // std::vector<PlotMatrix*> _plot_matrix_list;

    void updateInternalState();

    void getMaximumRangeX(double* minX, double* maxX);

    void buildData();

    PlotDataMap    _mapped_plot_data;

    void rearrangeGridLayout();

    QColor colorHint();

    void loadPlugins(QString subdir_name);

    std::map<QString,DataLoader*> data_loader;
    std::vector<StatePublisher*>  state_publisher;
    std::vector<DataStreamer*>    data_streamer;

    DataStreamer* _current_streamer;

    QDomDocument xmlSaveState();
    bool xmlLoadState(QDomDocument state_document);

    boost::circular_buffer<QDomDocument> _undo_states;
    boost::circular_buffer<QDomDocument> _redo_states;

    QElapsedTimer _undo_timer;

    QString _loaded_datafile;

    std::string _last_time_index_name;

    void createTabbedDialog(PlotMatrix *first_tab, bool undoable);

    void importPlotDataMap(const PlotDataMap &mapped_data);

protected:
    void mousePressEvent(QMouseEvent *event) ;

    void dragEnterEvent(QDragEnterEvent *event) ;

    void deleteLoadedData(const QString &curve_name);

    QTimer *_replot_timer;
signals:
    void requestRemoveCurveByName(const QString& name);

    void activateStreamingMode( bool active);

    void trackerTimeUpdated(QPointF point);

    void activateTracker(bool active);
};

#endif // MAINWINDOW_H
