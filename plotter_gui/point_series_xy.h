#ifndef POINT_SERIES_H
#define POINT_SERIES_H

#include <qwt_series_data.h>
#include "PlotJuggler/plotdata.h"

class PointSeriesXY: public QwtSeriesData<QPointF>
{
public:
    PointSeriesXY(const PlotData* y_axis, const PlotData* x_axis);

    virtual QPointF sample( size_t i ) const override;

    virtual QRectF boundingRect() const override;

    virtual size_t size() const override;

    nonstd::optional<QPointF> sampleFromTime(double t);

    void updateCache();

protected:

    QRectF _bounding_box;
    std::vector<QPointF> _cached_curve;

};

#endif // POINT_SERIES_H
