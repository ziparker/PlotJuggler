#include "plotmagnifier.h"
#include <qwt_plot.h>
#include <QDebug>
#include <limits>

PlotMagnifier::PlotMagnifier( QWidget *canvas) : QwtPlotMagnifier(canvas)
{
    for ( int axisId = 0; axisId < QwtPlot::axisCnt; axisId++ )
    {
        _lower_bounds[axisId] = std::numeric_limits<float>::min();
        _upper_bounds[axisId] = std::numeric_limits<float>::max();
    }
}

PlotMagnifier::~PlotMagnifier() {}

void PlotMagnifier::setAxisLimits(int axis, float lower, float upper)
{
    if ( axis >= 0 && axis < QwtPlot::axisCnt )
    {
        _lower_bounds[axis] = lower;
        _upper_bounds[axis] = upper;
    }
}

void PlotMagnifier::rescale( double factor )
{
    QwtPlot* plt = plot();
    if ( plt == NULL )
        return;

    factor = qAbs( factor );
    if ( factor == 1.0 || factor == 0.0 )
        return;

    bool doReplot = false;

    const bool autoReplot = plt->autoReplot();
    plt->setAutoReplot( false );

    for ( int axisId = 0; axisId < QwtPlot::axisCnt; axisId++ )
    {
        if ( isAxisEnabled( axisId ) )
        {
            const QwtScaleMap scaleMap = plt->canvasMap( axisId );

            double v1 = scaleMap.s1();
            double v2 = scaleMap.s2();

            if ( scaleMap.transformation() )
            {
                // the coordinate system of the paint device is always linear
                v1 = scaleMap.transform( v1 ); // scaleMap.p1()
                v2 = scaleMap.transform( v2 ); // scaleMap.p2()
            }

            const double center = 0.5 * ( v1 + v2 );
            const double width_2 = 0.5 * ( v2 - v1 ) * factor;

            v1 = center - width_2;
            v2 = center + width_2;

            if ( scaleMap.transformation() )
            {
                v1 = scaleMap.invTransform( v1 );
                v2 = scaleMap.invTransform( v2 );
            }

            if( v1 < _lower_bounds[axisId]) v1 = _lower_bounds[axisId];
            if( v2 > _upper_bounds[axisId]) v2 = _upper_bounds[axisId];

            plt->setAxisScale( axisId, v1, v2 );

            doReplot = true;
        }
    }

    plt->setAutoReplot( autoReplot );

    if ( doReplot ){
        plt->replot();
        emit rescaled();
    }
}
