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
#include "plotdata_qwt.h"
#include "customtracker.h"
#include <qwt_plot_legenditem.h>
#include <deque>
#include <QMessageBox>


class PlotWidget : public QwtPlot
{
    Q_OBJECT

public:
    PlotWidget(PlotDataMap* datamap, QWidget *parent=0);
    virtual ~PlotWidget();

    bool addCurve(const QString&  name, bool do_replot = true);
    void removeCurve(const QString& name);
    bool isEmpty();

    const std::map<QString, QwtPlotCurve*>& curveList();

    QDomElement xmlSaveState(QDomDocument &doc);
    bool xmlLoadState(QDomElement &element, QMessageBox::StandardButton* answer);

    QRectF maximumBoundingRect();
    QRectF currentBoundingRect();

    CurveTracker* tracker();

    void setScale( QRectF rect, bool emit_signal = true );

    void activateLegent(bool activate);

protected:
    virtual void dragEnterEvent(QDragEnterEvent *event) ;
    virtual void dragMoveEvent(QDragMoveEvent *event) ;
    virtual void dropEvent(QDropEvent *event) ;

    virtual void mousePressEvent(QMouseEvent *event) ;
    virtual void mouseReleaseEvent(QMouseEvent *event);

    virtual bool eventFilter(QObject *obj, QEvent *event);

signals:
    void swapWidgets(PlotWidget* source, PlotWidget* destination);
    void rectChanged(PlotWidget* self, QRectF rect );
    void plotModified();
    void trackerMoved(QPointF pos);

public slots:

    void replot() ;
    void detachAllCurves();  
    void zoomOut();
    void zoomOutHorizontal();
    void zoomOutVertical();

private slots:
    void launchRemoveCurveDialog();
    void canvasContextMenuTriggered(const QPoint &pos);
    void launchChangeColorDialog();
    void on_showPoints(bool checked);
    void on_externallyResized(QRectF new_rect);


private:
    std::map<QString, QwtPlotCurve*> _curve_list;

    QAction *_action_removeCurve;
    QAction *_action_removeAllCurves;
    QAction *_action_changeColors;
    QAction *_action_showPoints;
    QAction *_action_zoomOutHorizontally;
    QAction *_action_zoomOutVertically;


    QwtPlotZoomer* _zoomer;
    PlotMagnifier* _magnifier;
    QwtPlotPanner* _panner;
   // QRectF _prev_bounding;
    CurveTracker* _tracker;
    QwtPlotLegendItem* _legend;

private:

    void setAxisScale( int axisId, double min, double max, double step = 0 );

    PlotDataMap* _mapped_data;

    void buildActions();
    void buildLegend();

};

#endif
