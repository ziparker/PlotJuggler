#ifndef TABBEDPLOTWIDGET_H
#define TABBEDPLOTWIDGET_H

#include <QWidget>
#include <QTableWidget>
#include "plotmatrix.h"

namespace Ui {
class TabbedPlotWidget;
}

class MainWindow;


class TabbedPlotWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TabbedPlotWidget(PlotDataMap *mapped_data, MainWindow* main_window, QWidget *parent );

    PlotMatrix* currentPlotGrid();

    QTabWidget* tabWidget();

    void addTab();

    QDomElement xmlSaveState(QDomDocument &doc);
    bool xmlLoadState(QDomElement &tabbed_area);

    ~TabbedPlotWidget();

private slots:
    void on_pushAddColumn_pressed();

    void on_pushVerticalResize_pressed();

    void on_pushHorizontalResize_pressed();

    void on_pushAddRow_pressed();

    void on_addTabButton_pressed();

    void on_pushremoveEmpty_pressed();

    void on_tabWidget_currentChanged(int index);

    void on_tabWidget_tabCloseRequested(int index);

private:
    Ui::TabbedPlotWidget *ui;

    PlotDataMap *_mapped_data;

    MainWindow *_main_window;


signals:
    void undoableChangeHappened();
};

#endif // TABBEDPLOTWIDGET_H
