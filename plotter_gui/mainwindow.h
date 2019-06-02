#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <set>
#include <deque>
#include <functional>

#include <QCommandLineParser>
#include <QElapsedTimer>
#include <QMainWindow>
#include <QSignalMapper>
#include <QShortcut>

#include "plotwidget.h"
#include "plotmatrix.h"
#include "filterablelistwidget.h"
#include "tabbedplotwidget.h"
#include "subwindow.h"
#include "realslider.h"
#include "utils.h"
#include "PlotJuggler/dataloader_base.h"
#include "PlotJuggler/statepublisher_base.h"
#include "PlotJuggler/datastreamer_base.h"
#include "transforms/custom_function.h"

#include "ui_mainwindow.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(const QCommandLineParser& commandline_parser, QWidget *parent = nullptr);

    ~MainWindow();

    bool loadLayoutFromFile(QString filename);

    bool loadDataFromFiles(QStringList filenames );

    bool loadDataFromFile(QString filename, bool reuse_last_configuration = false );

public slots:
    void onUndoableChange();

    void onTrackerTimeUpdated(double absolute_time , bool do_replot);

    void onTrackerMovedFromWidget(QPointF pos );

    void onSplitterMoved(int, int);

    void resizeEvent(QResizeEvent *) ;

    void onPlotAdded(PlotWidget* plot);

    void onPlotMatrixAdded(PlotMatrix* matrix);

    void onActionSaveLayout();

    void onActionLoadLayout();

    void onActionLoadData();

    void onActionLoadStreamer(QString streamer_name);

    void onUndoInvoked();

    void onRedoInvoked();

    void on_tabbedAreaDestroyed(QObject*object);

    void onFloatingWindowDestroyed(QObject*object);

    void onCreateFloatingWindow(PlotMatrix* first_tab = nullptr);

    void onSwapPlots(PlotWidget* source, PlotWidget* destination);

    void on_pushButtonStreaming_toggled(bool streaming);

    void on_ToggleStreaming();

    void updateDataAndReplot(bool replot_hidden_tabs);

    void on_streamingSpinBox_valueChanged(int value);

    void onDeleteLoadedData();

    void on_actionAbout_triggered();

    void on_actionStopStreaming_triggered();

    void on_actionExit_triggered();

    void onTimeSlider_valueChanged(double abs_time);

    void onUpdateLeftTableValues();

    void deleteDataMultipleCurves(const std::vector<std::string> &curve_names);

    void on_pushButtonRemoveTimeOffset_toggled(bool checked);

    void on_pushButtonOptions_toggled(bool checked);

    void on_pushButtonActivateGrid_toggled(bool checked);

    void on_pushButtonRatio_toggled(bool checked);

    void on_pushButtonPlay_toggled(bool checked);

    void on_actionClearBuffer_triggered();

    void on_pushButtonUseDateTime_toggled(bool checked);

    void on_pushButtonTimeTracker_pressed();

    void on_minimizeView();

    void addMathPlot(const std::string &linked_name);

    void editMathPlot(const std::string &plot_name);

    void onRefreshMathPlot(const std::string &plot_name);

    void updateTimeSlider();

    void updateTimeOffset();

    void buildDummyData();

    void on_actionFunction_editor_triggered();

    void publishPeriodically();

    void on_actionReportBug_triggered();

    void on_actionCheatsheet_triggered();

    void on_actionSupportPlotJuggler_triggered();

    void on_actionSaveAllPlotTabs_triggered();

private:

    Ui::MainWindow *ui;

    TabbedPlotWidget *_main_tabbed_widget;

    QShortcut _undo_shortcut;
    QShortcut _redo_shortcut;
    QShortcut _minimize_view;
    QShortcut _toggle_streaming;
    QShortcut _toggle_playback;

    bool _minimized;

    void createActions();

    FilterableListWidget* _curvelist_widget;

    void updatedDisplayTime();

    void forEachWidget(std::function<void(PlotWidget*, PlotMatrix*, int, int)> op);

    void forEachWidget(std::function<void(PlotWidget*)> op);

    PlotDataMapRef  _mapped_plot_data;

    CustomPlotMap _custom_plots;

    void rearrangeGridLayout();

    void loadPlugins(QString subdir_name);

    std::map<QString,DataLoader*>      _data_loader;

    std::map<QString,StatePublisher*>  _state_publisher;

    std::map<QString,DataStreamer*>    _data_streamer;

    DataStreamer* _current_streamer;

    QDomDocument xmlSaveState() const;

    bool xmlLoadState(QDomDocument state_document);

    void checkAllCurvesFromLayout(const QDomElement& root);

    std::deque<QDomDocument> _undo_states;

    std::deque<QDomDocument> _redo_states;

    QElapsedTimer _undo_timer;

    bool _disable_undo_logging;

    bool _test_option;

    bool _autostart_publishers;

    double _tracker_time;

    std::vector<FileLoadInfo> _loaded_datafiles;

    QSignalMapper *_streamer_signal_mapper;

    void createTabbedDialog(QString suggest_win_name, PlotMatrix *first_tab);

    void importPlotDataMap(PlotDataMapRef &new_data, bool remove_old);

    bool isStreamingActive() const ;

    CurveTracker::Parameter _tracker_param;

    std::map<CurveTracker::Parameter, QIcon> _tracker_button_icons;

    void closeEvent(QCloseEvent *event);

    void loadPluginState(const QDomElement &root);
    
    QDomElement savePluginState(QDomDocument &doc);

    std::tuple<double,double,int> calculateVisibleRangeX();

    void addOrEditMathPlot(const std::string &name, bool edit);
    
    void deleteAllDataImpl();

    void updateRecentDataMenu(QStringList new_filenames);

    void updateRecentLayoutMenu(QStringList new_filenames);

protected:

    MonitoredValue _time_offset;

    QTimer *_replot_timer;

    QTimer *_publish_timer;

    QDateTime _prev_publish_time;

signals:
    void requestRemoveCurveByName(const std::string& name);

    void activateStreamingMode( bool active);

    void activateTracker(bool active);

};



#endif // MAINWINDOW_H
