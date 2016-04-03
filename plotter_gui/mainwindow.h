#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "plotwidget.h"
#include "plotmatrix.h"
#include "../plugins/dataloader_base.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:

 //   void swapWidgets(QString src, QString dst);

    void on_splitter_splitterMoved(int, int);
    void resizeEvent(QResizeEvent *) ;

    void on_lineEdit_textChanged(const QString &search_string);

    void on_radioRegExp_toggled(bool checked);

    void on_checkBoxCaseSensitive_toggled(bool checked);

    void on_pushAddRow_pressed();

    void on_pushAddColumn_pressed();

    void createActions();

    void on_pushremoveEmpty_pressed();

    void on_horizontalSlider_valueChanged(int value);

    void on_plotAdded(PlotWidget* widget);
    void addCurveToPlot(QString curve_name, PlotWidget* destination);
    void on_addTabButton_pressed();

    void onActionSaveLayout();
    void onActionLoadLayout();
    void onActionLoadDataFile();

    void on_pushButton_toggled(bool checked);

    void on_radioFlatView_toggled(bool checked);

    void on_radioTreeView_toggled(bool checked);

    void on_radioContains_toggled(bool checked);

    void on_pushHorizontalResize_pressed();

    void on_pushVerticalResize_pressed();

    void on_pushLinkHorizontalScale_toggled(bool checked);

    void on_pushButtonActivateTracker_toggled(bool checked);

private:
    Ui::MainWindow *ui;

    QAction *deleteOneAct;
    QAction *deleteAllAct;

    PlotMatrix *currentPlotGrid();

    void buildData();

    //std::map<QString, SharedVector> _mapped_raw_data;
    std::map<QString, PlotData*>    _mapped_plot_data;

    void rearrangeGridLayout();
    QVector< QWidget*> settings_widgets;

    bool _horizontal_link;

    QColor colorHint();

    void loadDataPlugins(QString subdir_name);

    std::map<QString,DataLoader*> data_loader;

protected:
    void mousePressEvent(QMouseEvent *event) ;
    void contextMenuEvent(QContextMenuEvent *event) ;

    void dragEnterEvent(QDragEnterEvent *event) ;
    void dragMoveEvent(QDragMoveEvent *event) ;
    void dropEvent(QDropEvent *event) ;


    void deleteLoadedData();
};

#endif // MAINWINDOW_H
