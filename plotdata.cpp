#include "plotdata.h"

PlotData::PlotData()
{

}

size_t PlotData::size() const
{
    return 1;
}

QPointF PlotData::sample(size_t i) const
{
    return QPointF(0,i);
}

QRectF PlotData::boundingRect() const
{
    return QRectF( 0,1,1,1);
}

