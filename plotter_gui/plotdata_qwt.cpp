#include "plotdata_qwt.h"
#include <limits>
#include <stdexcept>


PlotDataQwt::PlotDataQwt(PlotDataPtr base):
    _plot_data(base),
    _subsample(1),
    _transform( noTransform )
{

}

QPointF PlotDataQwt::sample(size_t i) const
{
  if(_transform == noTransform) {
    const auto& p = _plot_data->at( i );
    return QPointF( p.x, p.y) ;
  }
  else if(_transform == firstDerivative) {

    const auto& p0 = _plot_data->at( i );
    const auto& p1 = _plot_data->at( i+1 );
    const auto delta = p1.x - p0.x;
    const auto vel = (p1.y - p0.y) /delta;
    return QPointF( (p1.x + p0.x) /2.0, vel) ;
  }
  else // if(_transform == secondDerivative)
  {
    const auto& p = _plot_data->at( i );
    return QPointF( p.x, p.y) ;
  }
}

QRectF PlotDataQwt::boundingRect() const
{
    qDebug() << "boundingRect not implemented";
    return QRectF(0,0,1,1);
}

QRectF PlotDataQwt::maximumBoundingRect(double min_X, double max_X)
{
    int x1 = _plot_data->getIndexFromX( min_X );
    int x2 = _plot_data->getIndexFromX( max_X );

    if( x1 <0 || x2 <0){
      return QRectF();
    }

    auto range_X_opt = getRangeX();

    if( !range_X_opt ){
      return QRectF();
    }

    auto range_Y_opt = getRangeY( x1, x2  );
    if( !range_Y_opt){
      return QRectF();
    }

    auto range_X = range_X_opt.get();
    auto range_Y = range_Y_opt.get();

    QRectF rect ( range_X.min,  range_Y.min,
                  range_X.max - range_X.min,
                  range_Y.max - range_Y.min );
    return rect;
}

size_t PlotDataQwt::size() const
{
    //return (_index_last - _index_first +1) / _subsample;
   // return _plot_data->size() /  _subsample ;
   if( _plot_data->size() <=1 ) return 0;
   return _plot_data->size() -1;
}

QColor PlotDataQwt::randomColorHint() const
{   
     return _plot_data->getColorHint() ;
}

void PlotDataQwt::setColorHint(QColor color)
{
     _plot_data->setColorHint(color) ;
}

void PlotDataQwt::setSubsampleFactor()
{
  //  _subsample = (_plot_data->size() / 2000) + 1;
}

PlotData::RangeTime PlotDataQwt::getRangeX()
{
  // std::lock_guard<std::mutex> lock(_mutex);
  if( this->size() < 2 )
    return  PlotData::RangeTime() ;
  else
    return  PlotData::RangeTime( { sample(0).x(), sample( this->size() -1).x() } );
}




PlotData::RangeValue PlotDataQwt::getRangeY(int first_index, int last_index)
{
  if( first_index < 0 || last_index < 0 || first_index > last_index)
  {
    return PlotData::RangeValue();
  }
  //std::lock_guard<std::mutex> lock(_mutex);

  const double first_Y = sample(first_index).y();
  double y_min = first_Y;
  double y_max = first_Y;

  for (int i = first_index+1; i < last_index; i++)
  {
    const double Y = sample(i).y();

    if( Y < y_min )      y_min = Y;
    else if( Y > y_max ) y_max = Y;
  }
  return PlotData::RangeValue( { y_min, y_max } );
}
