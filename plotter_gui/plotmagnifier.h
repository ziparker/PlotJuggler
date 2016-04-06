#ifndef PLOTMAGNIFIER_H
#define PLOTMAGNIFIER_H

#include <QTimer>
#include <qwt_plot_magnifier.h>
#include <qwt_plot.h>

class PlotMagnifier : public QwtPlotMagnifier
{
    Q_OBJECT


public:
    explicit PlotMagnifier( QWidget *canvas);
    virtual ~PlotMagnifier();

    void setAxisLimits(int axis,float lower, float upper);


protected:
    virtual void rescale( double factor ) ;
   // virtual void widgetMouseMoveEvent( QMouseEvent *event );
    virtual void widgetWheelEvent( QWheelEvent *event );

    float _lower_bounds[QwtPlot::axisCnt];
    float _upper_bounds[QwtPlot::axisCnt];

    QPointF _mouse_position;

signals:
    void rescaled(QRectF new_size);

private:
    QPointF invTransform(QPoint pos);
    QTimer _future_emit;

};

#endif // PLOTMAGNIFIER_H
