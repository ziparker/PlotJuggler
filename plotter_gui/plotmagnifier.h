#ifndef PLOTMAGNIFIER_H
#define PLOTMAGNIFIER_H

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
    void rescale( double factor ) ;

    float _lower_bounds[QwtPlot::axisCnt];
    float _upper_bounds[QwtPlot::axisCnt];

signals:
    void rescaled();

};

#endif // PLOTMAGNIFIER_H
