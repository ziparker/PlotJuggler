#include "point_series_xy.h"


PointSeriesXY::PointSeriesXY(const PlotData *y_axis, const PlotData *x_axis)
{

}

QPointF PointSeriesXY::sample(size_t i) const
{
    return QPointF();
}

QRectF PointSeriesXY::boundingRect() const
{
    return _bounding_box;
}

size_t PointSeriesXY::size() const
{
    return _cached_curve.size();
}

nonstd::optional<QPointF> PointSeriesXY::sampleFromTime(double t)
{
    return nonstd::optional<QPointF>();
}

void PointSeriesXY::updateCache()
{

}
