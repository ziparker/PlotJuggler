#ifndef SUBWINDOW_H
#define SUBWINDOW_H

#include <QMainWindow>
#include "tabbedplotwidget.h"

class SubWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit SubWindow(PlotDataMap *mapped_data, QWidget *parent = 0);

    TabbedPlotWidget* tabbedWidget() { return &tabbed_widget_;}

signals:
    void closeRequestedByUser();

protected:
    virtual void closeEvent(QCloseEvent *event);
    TabbedPlotWidget tabbed_widget_;

};

#endif // SUBWINDOW_H
