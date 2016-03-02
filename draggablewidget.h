#ifndef DragableWidget_H
#define DragableWidget_H

#include <map>
#include <QObject>
#include <QTextEdit>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_grid.h>
#include <qwt_symbol.h>
#include <qwt_legend.h>
#include "plotdata.h"

class DragableWidget : public QwtPlot
{
    Q_OBJECT

public:
    DragableWidget(QWidget *parent=0);

    void addCurve(const QString&  name, PlotData* data);
    void removeCurve(const QString& name);

protected:
    void dragEnterEvent(QDragEnterEvent *event) Q_DECL_OVERRIDE;
    void dragMoveEvent(QDragMoveEvent *event) Q_DECL_OVERRIDE;
    void dropEvent(QDropEvent *event) Q_DECL_OVERRIDE;
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

signals:
    void swapWidgets(QString s, QString to);

private:
    std::map<QString, PlotData*> curve;

};

#endif // DragableWidget_H
