#ifndef PLOTMAGNIFIER_H
#define PLOTMAGNIFIER_H

#include <qwt_plot_magnifier.h>
#include <qwt_plot.h>

class PlotMagnifier : public QwtPlotMagnifier
{

public:
    explicit PlotMagnifier( QWidget *canvas);
    virtual ~PlotMagnifier();

    void setAxisLimits(int axis,float lower, float upper);


protected:
    void rescale( double factor ) Q_DECL_OVERRIDE;

    float _lower_bounds[QwtPlot::axisCnt];
    float _upper_bounds[QwtPlot::axisCnt];
};

#endif // PLOTMAGNIFIER_H
