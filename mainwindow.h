#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "plotwidget.h"
#include "plotmatrix.h"

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
    void resizeEvent(QResizeEvent *) Q_DECL_OVERRIDE;

    void on_lineEdit_textChanged(const QString &arg1);

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

private:
    Ui::MainWindow *ui;

    QAction *deleteOneAct;
    QAction *deleteAllAct;

    PlotMatrix *currentPlotGrid();

    void buildData();

    PlotDataMap _mapped_plot_data;
    void rearrangeGridLayout();

protected:
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void contextMenuEvent(QContextMenuEvent *event) Q_DECL_OVERRIDE;

    void dragEnterEvent(QDragEnterEvent *event) Q_DECL_OVERRIDE;
    void dragMoveEvent(QDragMoveEvent *event) Q_DECL_OVERRIDE;
    void dropEvent(QDropEvent *event) Q_DECL_OVERRIDE;

};

#endif // MAINWINDOW_H
