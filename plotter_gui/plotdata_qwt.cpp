#include "plotdata_qwt.h"
#include <limits>
#include <stdexcept>



<<<<<<< HEAD
PlotDataQwt::PlotDataQwt(PlotDataPtr base):
    _plot_data(base),
    _subsample(1)
  //   _index_first( 0 ),
  //   _index_last ( _x_points->size() -1 )
{

}

=======

PlotDataQwt::PlotDataQwt(const PlotData& other):
    PlotData(other) //,
  //  _subsample(1),
  //   _index_first( 0 ),
  //   _index_last ( _x_points->size() -1 )
{

}
>>>>>>> b821eb0b426188e0e1635851b6eb4b29d82b171e


QPointF PlotDataQwt::sample(size_t i) const
{
    //   size_t index = i*_subsample +_index_first;
    //   if( index > _index_last) index =_index_last;
<<<<<<< HEAD
    auto p = _plot_data->at( i*_subsample );
    QPointF point( p.first, p.second ) ;
=======

    QPointF point( _x_points->at(i), _y_points->at(i) ) ;
>>>>>>> b821eb0b426188e0e1635851b6eb4b29d82b171e
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
<<<<<<< HEAD
    auto range_X = _plot_data->getRangeX();
    auto range_Y = _plot_data->getRangeY();

    QRectF rect ( range_X.min, range_Y.min,
                  range_X.max - range_X.min,
                  range_Y.max - range_Y.min );
=======
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

>>>>>>> b821eb0b426188e0e1635851b6eb4b29d82b171e
    return rect;
}

size_t PlotDataQwt::size() const
{
    //return (_index_last - _index_first +1) / _subsample;
<<<<<<< HEAD
    return _plot_data->size() /  _subsample ;
=======
    return _x_points->size();
>>>>>>> b821eb0b426188e0e1635851b6eb4b29d82b171e
}


/*void PlotDataQwt::setRangeX(double x_left, double x_right)
{
<<<<<<< HEAD
     boost::circular_buffer<double>::iterator lower, upper;
=======
    /*  boost::circular_buffer<double>::iterator lower, upper;
>>>>>>> b821eb0b426188e0e1635851b6eb4b29d82b171e

    lower = std::lower_bound(_x_points->begin(), _x_points->end(), x_left );
    upper = std::upper_bound(_x_points->begin(), _x_points->end(), x_right );

    _index_first = std::distance( _x_points->begin(), lower);
    _index_last  = std::distance( _x_points->begin(), upper) -1;

    _subsample = (_index_last - _index_first)  / 5000;
    if( _subsample < 1) _subsample = 1;
<<<<<<< HEAD
}*/
=======
*/
}
>>>>>>> b821eb0b426188e0e1635851b6eb4b29d82b171e

QColor PlotDataQwt::colorHint() const
{
    int red, green, blue;
     _plot_data->getColorHint(&red, &green, &blue) ;

    return QColor( red, green, blue);
}

void PlotDataQwt::setColorHint(QColor color)
{
     _plot_data->setColorHint(
                 color.red(), color.green(), color.blue()) ;
}

void PlotDataQwt::setSubsampleFactor()
{
    _subsample = (_plot_data->size() / 2000) + 1;
}



