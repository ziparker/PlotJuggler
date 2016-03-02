#include "plotdata.h"
#include <limits>


PlotData::PlotData(int capacity)
{
    _subsample = 1;
    reserve(capacity);

    y_min = std::numeric_limits<float>::max();
    y_max = std::numeric_limits<float>::min();
}

void PlotData::reserve(size_t capacity)
{
    _raw_points.reserve(capacity);
}

void PlotData::pushBack(QPointF point)
{
    _raw_points.push_back( point );

    if( y_min > point.y()) y_min = point.y();
    if( y_max < point.y()) y_max = point.y();
}

void PlotData::setSubsampleFactor(int factor)
{
    if( factor < 1) factor = 1;
    _subsample = factor;
}

size_t PlotData::size() const
{
    return _raw_points.size() / _subsample;
}

QPointF PlotData::sample(size_t i) const
{
    QPointF point = _raw_points.at(i*_subsample);
    //qDebug() << point.x() << " / " << point.y();
    return point;
}

QRectF PlotData::boundingRect() const
{
    float left  = _raw_points[0].x();
    float right = _raw_points[ _raw_points.size() -1].x();
    float height = y_max - y_min;

    return QRectF(  left, y_min - height*0.02, (right - left), height*1.04 ) ;
}

