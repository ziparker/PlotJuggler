#ifndef SERIES_DATA_H
#define SERIES_DATA_H

#include <qwt_series_data.h>
#include "PlotJuggler/plotdata.h"

class DataSeriesBase: public QwtSeriesData<QPointF>
{

public:
    DataSeriesBase(double time_offset):
        _time_offset(time_offset){}


    virtual QPointF sample( size_t i ) const override
    {
        return _cached_curve[i];
    }

    QRectF boundingRect() const override
    {
        return _bounding_box;
    }

    virtual size_t size() const override
    {
        return _cached_curve.size();
    }

    virtual PlotData::RangeValueOpt getVisualizationRangeY(PlotData::RangeTime range_X) = 0;

    virtual nonstd::optional<QPointF> sampleFromTime(double t) = 0;

    virtual bool updateCache() = 0;

    PlotData::RangeTimeOpt getVisualizationRangeX()
    {
        if( this->size() < 2 )
            return  PlotData::RangeTimeOpt();
        else{
            return PlotData::RangeTimeOpt( { _bounding_box.left(), _bounding_box.right() } );
        }
    }

public slots:

    virtual void onTimeoffsetChanged(double offset)
    {
        _time_offset = offset;
    }

protected:

    std::vector<QPointF> _cached_curve;

    QRectF _bounding_box;

    double _time_offset;
};

#endif // SERIES_DATA_H
