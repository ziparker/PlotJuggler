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
#include <qwt_plot_legenditem.h>



class PlotWidget : public QwtPlot
{
    Q_OBJECT

public:
    PlotWidget( QWidget *parent=0);
    virtual ~PlotWidget();

    void addCurve(const QString&  name, PlotData* data);
    void removeCurve(const QString& name);
    bool isEmpty();

    const std::map<QString, QwtPlotCurve*>& curveList();

    void detachAllCurves();
    QDomElement getDomElement(QDomDocument &doc);

    QRectF maximumBoundingRect();
    QRectF currentBoundingRect();

    CurveTracker* tracker();

    void setScale( QRectF rect );

    void undoScaleChange();
    void redoScaleChange();

protected:
    virtual void dragEnterEvent(QDragEnterEvent *event) ;
    virtual void dragMoveEvent(QDragMoveEvent *event) ;
    virtual void dropEvent(QDropEvent *event) ;

    virtual void mousePressEvent(QMouseEvent *event) ;
    virtual void mouseReleaseEvent(QMouseEvent *event);

    virtual void keyPressEvent(QKeyEvent *event);
    virtual void keyReleaseEvent(QKeyEvent *event);


signals:
    void swapWidgets(QString s, QString to);
    void curveNameDropped(QString curve_name, PlotWidget* destination);
    void horizontalScaleChanged(QRectF bound);

public Q_SLOTS:
    void replot() ;

private slots:
    void launchRemoveCurveDialog();
    void canvasContextMenuTriggered(const QPoint &pos);
    void launchChangeColorDialog();
    void on_showPoints(bool checked);

private:
    std::map<QString, QwtPlotCurve*> _curve_list;

    QAction *removeCurveAction;
    QAction *changeColorsAction;
    QAction *showPointsAction;

    QwtPlotZoomer* _zoomer;
    PlotMagnifier* _magnifier;
    QwtPlotPanner* _panner;
    QRectF _prev_bounding;
    CurveTracker* _tracker;
    QwtPlotLegendItem* _legend;

private:
    void setHorizontalAxisRange(float min, float max);
    void setVerticalAxisRange(float min, float max);
    void setAxisScale( int axisId, double min, double max, double step = 0 );

};

#endif
