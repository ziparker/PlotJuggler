#include "plotdata_qwt.h"
#include <limits>
#include <stdexcept>

/*PlotDataQwt::PlotDataQwt(PlotData& base)
{
   //TODO FIXME *this = base;
}*/


PlotDataQwt::PlotDataQwt(SharedVector x, SharedVector y, std::string name)
{
    this->addData(x,y);
    this->setName(name);
}

QPointF PlotDataQwt::sample(size_t i) const
{
    int index = i +_index_first;
    QPointF point( (*_x_points)[index], (*_y_points)[index] ) ;
    return point ;
}

QRectF PlotDataQwt::boundingRect() const
{
    QRectF rect(  x_min, y_min, (x_max - x_min),  y_max - y_min );
    return rect ;
}

size_t PlotDataQwt::size() const
{
    return PlotData::size();
}

QColor PlotDataQwt::colorHint() const
{
    return _color;
}

void PlotDataQwt::setColorHint(QColor color)
{
    _color = color;
}



