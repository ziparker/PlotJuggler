#include "plotdata_qwt.h"
#include <limits>
#include <stdexcept>


PlotDataQwt::PlotDataQwt(PlotDataPtr base):
    _plot_data(base),
    _subsample(1)
{

}



QPointF PlotDataQwt::sample(size_t i) const
{
    //   size_t index = i*_subsample +_index_first;
    //   if( index > _index_last) index =_index_last;
    auto p = _plot_data->at( i*_subsample );
    QPointF point( p.x, p.y ) ;

    return point ;
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

    auto range_X_opt = _plot_data->getRangeX();

    if( !range_X_opt ){
      return QRectF();
    }

    auto range_Y_opt = _plot_data->getRangeY( x1, x2  );
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
    return _plot_data->size() /  _subsample ;
}

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
  //  _subsample = (_plot_data->size() / 2000) + 1;
}



