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
#include "plotmagnifier.h"
#include <qwt_plot_zoomer.h>
#include <qwt_plot_panner.h>
#include <QDomDocument>
#include "plotdata.h"
#include "customtracker.h"

typedef std::map<QString, PlotData*> PlotDataMap;

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
    QDomElement getDomElement(QDomDocument &doc);

    typedef enum{
        ZOOM_MODE,
        DRAG_N_DROP_MODE
    }PlotWidgetMode;

    void setMode(PlotWidgetMode mode);

    void setHorizontalAxisRange(float min, float max);
    void setVerticalAxisRange(float min, float max);

    QRectF maximumBoundingRect();
    QRectF currentBoundingRect();

    CurveTracker* tracker();

protected:
    void dragEnterEvent(QDragEnterEvent *event) Q_DECL_OVERRIDE;
    void dragMoveEvent(QDragMoveEvent *event) Q_DECL_OVERRIDE;
    void dropEvent(QDropEvent *event) Q_DECL_OVERRIDE;
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void contextMenuEvent(QContextMenuEvent *event) Q_DECL_OVERRIDE;

signals:
    void swapWidgets(QString s, QString to);
    void curveNameDropped(QString curve_name, PlotWidget* destination);
    void horizontalScaleChanged(QRectF bound);

public Q_SLOTS:
    void replot() Q_DECL_OVERRIDE;

private slots:
    void launchRemoveCurveDialog();

private:
    std::map<QString, QwtPlotCurve*> _curve_list;
    QAction *removeCurveAction;
    PlotWidgetMode _mode;
    QwtPlotZoomer* _zoomer;
    PlotMagnifier* _magnifier;
    QwtPlotPanner* _panner;
    QRectF _prev_bounding;
    CurveTracker* _tracker;

};

#endif // DragableWidget_H
