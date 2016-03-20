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

    _indexA = 0;
    _indexB = _raw_points.size() -1;

    x_min  = _raw_points[0].x();
    x_max  = (_raw_points.back().x()) ;
}

void PlotData::setSubsampleFactor(int factor)
{
    if( factor < 1) factor = 10;
    _subsample = factor;
}

size_t PlotData::size() const
{
    // return _raw_points.size() ;
   return _indexB - _indexA;
}

QPointF PlotData::sample(size_t i) const
{
    if( i >= _raw_points.size() ) return _raw_points.back() ;

    //qDebug() << point.x() << " / " << point.y();
   // return _raw_points.at(i );;
    return _raw_points.at(i + _indexA );
}

QRectF PlotData::boundingRect() const
{
    //float left  = _raw_points[0].x();
   // float right = _raw_points[ _raw_points.size() -1].x();
    QRectF rect(  x_min, y_min, (x_max - x_min),  y_max - y_min );
   // qDebug() << rect;
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

void PlotData::setRangeX(float t_center, float t_range)
{
    float t_min = t_center - t_range/2;
    float t_max = t_center + t_range/2;

    std::vector<QPointF>::const_iterator lower = std::lower_bound(_raw_points.begin(), _raw_points.end(), QPointF(t_min,t_min), compareQPointF);
    std::vector<QPointF>::const_iterator upper = std::upper_bound(_raw_points.begin(), _raw_points.end(), QPointF(t_max,t_max), compareQPointF);

 //   QPointF point_left  = *lower;
 //   QPointF point_right = *upper;
  _indexA = ( lower - _raw_points.begin());
  _indexB = ( upper - _raw_points.begin()-1);

    x_min = t_min;
    x_max = t_max;

    //qDebug() <<  t_min << " " << t_max << " / " <<_indexA << " " << _indexB;
}

