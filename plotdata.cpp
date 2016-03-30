#include "plotdata.h"
#include <limits>
#include <QMenu>
#include <stdexcept>

PlotData::PlotData()
{
    _subsample = 1;
    _index_first = 0;
    _index_last  = 0;
}

PlotData::PlotData(SharedVector x, SharedVector y)
{
    _subsample = 1;
    _index_first = 0;
    _index_last  = 0;
    addData(x,y);
}

void PlotData::addData(SharedVector x, SharedVector y)
{
    _x_points = x;
    _y_points = y;

    if( x->size() != y->size() )
    {
        throw std::runtime_error("size of x and y vectors must match");
    }

    y_min = std::numeric_limits<double>::max();
    y_max = std::numeric_limits<double>::min();

    x_min = _x_points->front();
    x_max = _x_points->back();

    for (int i=0; i< x->size(); i++)
    {
        double Y = _y_points->at(i);
        if( Y < y_min ) y_min = Y;
        if( Y > y_max ) y_max = Y;
    }
    _index_first = 0;
    _index_last  = _x_points->size() -1;
}



void PlotData::setSubsampleFactor(int factor)
{
    if( factor < 1) factor = 10;
    _subsample = factor;
}

size_t PlotData::size() const
{
    // return _raw_points.size() ;
   return _index_last - _index_first + 1;
}

QPointF PlotData::sample(size_t i) const
{
    int index = i +_index_first;
    QPointF point( (*_x_points)[index], (*_y_points)[index] ) ;
    return point ;
}

QRectF PlotData::boundingRect() const
{
    QRectF rect(  x_min, y_min, (x_max - x_min),  y_max - y_min );
    return rect ;
}

QColor PlotData::colorHint()
{
    return _color;
}

void PlotData::setColorHint(QColor color)
{
    _color = color;
}

bool compareQPointF(const QPointF &a, const QPointF &b)
{
     return ( a.x() < b.x() );
}

void PlotData::setRangeX(double t_center, double t_range)
{
    double t_min = t_center - t_range/2;
    double t_max = t_center + t_range/2;

    std::vector<double>::const_iterator lower, upper;

    lower = std::lower_bound(_x_points->begin(), _x_points->end(), t_min );
    upper = std::upper_bound(_x_points->begin(), _x_points->end(), t_max );

    _index_first = ( lower - _x_points->begin());
    _index_last  = ( upper - _x_points->begin());

    x_min = t_min;
    x_max = t_max;
}

int PlotData::getIndexAtPositionX(double x) const
{

}

