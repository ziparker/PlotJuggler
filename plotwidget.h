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

typedef std::map<QString, PlotData> PlotDataMap;

class PlotWidget : public QwtPlot
{
    Q_OBJECT

public:
    PlotWidget( QWidget *parent=0);
    virtual ~PlotWidget();

    void addCurve(const QString&  name, PlotData* data);
    void removeCurve(const QString& name);
    bool isEmpty();

    void detachAllCurves();
protected:
    void dragEnterEvent(QDragEnterEvent *event) Q_DECL_OVERRIDE;
    void dragMoveEvent(QDragMoveEvent *event) Q_DECL_OVERRIDE;
    void dropEvent(QDropEvent *event) Q_DECL_OVERRIDE;
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void contextMenuEvent(QContextMenuEvent *event) Q_DECL_OVERRIDE;

signals:
    void swapWidgets(QString s, QString to);
    void curveNameDropped(QString curve_name, PlotWidget* destination);

private:
   // std::map<QString, QwtPlotCurve*> _curve_list;


};

#endif // DragableWidget_H
