#include "plotdata_qwt.h"
#include <limits>
#include <stdexcept>

/*PlotDataQwt::PlotDataQwt(PlotData& base)
{
   //TODO FIXME *this = base;
}*/


PlotDataQwt::PlotDataQwt(SharedVector x, SharedVector y, std::string name):
    PlotData( x, y, name),
    _subsample(1),
    _index_first( 0 ),
    _index_last ( x->size() -1 )
{

}

PlotDataQwt::PlotDataQwt(const PlotDataQwt & other):
    PlotData( other.getVectorX(), other.getVectorY(),  other.name() ),
    _subsample(1),
    _index_first( 0 ),
    _index_last ( PlotData::size() -1 )
{

}

QPointF PlotDataQwt::sample(size_t i) const
{
    int index = i*_subsample +_index_first;
    QPointF point( (*_x_points)[index], (*_y_points)[index] ) ;
    return point ;
}

QRectF PlotDataQwt::boundingRect() const
{
    double x_min = (*_x_points)[ _index_first ];
    double x_max = (*_x_points)[ _index_last ];

    QRectF rect(  x_min,
                  _y_min,
                  x_max - x_min,
                  _y_max - _y_min );
    return rect ;
}

QRectF PlotDataQwt::maximumBoundingRect() const
{
    QRectF rect(  _x_min,
                  _y_min,
                  _x_max - _x_min,
                  _y_max - _y_min );
    return rect ;
}

size_t PlotDataQwt::size() const
{
    return (_index_last - _index_first +1) / _subsample;
}


void PlotDataQwt::setRangeX(double x_left, double x_right)
{
    std::vector<double>::iterator lower, upper;

    lower = std::lower_bound(_x_points->begin(), _x_points->end(), x_left );
    upper = std::upper_bound(_x_points->begin(), _x_points->end(), x_right );

    _index_first = std::distance( _x_points->begin(), lower);
    _index_last  = std::distance( _x_points->begin(), upper) -1;

    _subsample = (_index_last-_index_first)  / 5000;
    if( _subsample < 1) _subsample = 1;

   // qDebug() << "new range " << _index_first << " " << _index_last <<" " << _subsample;
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



