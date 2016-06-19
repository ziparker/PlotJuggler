#include "plotdata_qwt.h"
#include <limits>
#include <stdexcept>

/*PlotDataQwt::PlotDataQwt(PlotData& base)
{
   //TODO FIXME *this = base;
}*/



PlotDataQwt::PlotDataQwt(const PlotData& other):
    PlotData(other) //,
  //  _subsample(1),
  //   _index_first( 0 ),
  //   _index_last ( _x_points->size() -1 )
{

}


QPointF PlotDataQwt::sample(size_t i) const
{
    //   size_t index = i*_subsample +_index_first;
    //   if( index > _index_last) index =_index_last;

    QPointF point( _x_points->at(i), _y_points->at(i) ) ;
    return point ;
}

QRectF PlotDataQwt::boundingRect() const
{
    qDebug() << "boundingRect";

    return QRectF(0,0,1,1);
    /*double x_min = _x_points->at( _index_first );
    double x_max = _x_points->at( _index_last );

    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::min();

    for (size_t i=_index_first; i<= _index_last; i += _subsample )
    {
        double Y = _y_points->at(i);
        if( Y < y_min ) y_min = Y;
        if( Y > y_max ) y_max = Y;
    }

    return QRectF (  x_min,  y_min,  x_max - x_min,  y_max - y_min );*/
}

QRectF PlotDataQwt::maximumBoundingRect()
{
    double x_min =  _x_points->front();
    double x_max =  _x_points->back();

    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::min();

    for (size_t i=0; i< _y_points->size(); i ++)
    {
        double Y = _y_points->at(i);
        if( Y < y_min ) y_min = Y;
        if( Y > y_max ) y_max = Y;
    }

    QRectF rect ( x_min,  y_min,  x_max - x_min,  y_max - y_min );
    qDebug() << rect;

    return rect;
}

size_t PlotDataQwt::size() const
{
    //return (_index_last - _index_first +1) / _subsample;
    return _x_points->size();
}


void PlotDataQwt::setRangeX(double x_left, double x_right)
{
    /*  boost::circular_buffer<double>::iterator lower, upper;

    lower = std::lower_bound(_x_points->begin(), _x_points->end(), x_left );
    upper = std::upper_bound(_x_points->begin(), _x_points->end(), x_right );

    _index_first = std::distance( _x_points->begin(), lower);
    _index_last  = std::distance( _x_points->begin(), upper) -1;

    _subsample = (_index_last - _index_first)  / 5000;
    if( _subsample < 1) _subsample = 1;
*/
}

QColor PlotDataQwt::colorHint() const
{
    return QColor( _color_hint_red, _color_hint_green, _color_hint_blue);
}

void PlotDataQwt::setColorHint(QColor color)
{
    _color_hint_red   = color.red();
    _color_hint_green = color.green();
    _color_hint_blue  = color.blue();
}



