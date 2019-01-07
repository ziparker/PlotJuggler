#ifndef SERIES_DATA_H
#define SERIES_DATA_H

#include <qwt_series_data.h>
#include "PlotJuggler/plotdata.h"

class DataSeriesBase: public QwtSeriesData<QPointF>
{

public:
    DataSeriesBase(const PlotData* transformed):
        _transformed_data(transformed),
        _time_offset(0)
    {}

    virtual QPointF sample( size_t i ) const override
    {
        const auto& p = _transformed_data->at(i);
        return QPointF(p.x - _time_offset, p.y);
    }

    virtual size_t size() const override
    {
        return _transformed_data->size();
    }

    QRectF boundingRect() const override final
    {
        QRectF box = _bounding_box;
        box.setLeft(  _bounding_box.left()  - _time_offset );
        box.setRight( _bounding_box.right() - _time_offset );
        return box;
    }

    void setTimeOffset(double offset)
    {
        _time_offset = offset;
    }

    virtual PlotData::RangeValueOpt getVisualizationRangeY(PlotData::RangeTime range_X) = 0;

    virtual nonstd::optional<QPointF> sampleFromTime(double t) = 0;

    virtual bool updateCache() = 0;

    PlotData::RangeTimeOpt getVisualizationRangeX()
    {
        if( this->size() < 2 )
            return  PlotData::RangeTimeOpt();
        else{
            return PlotData::RangeTimeOpt( { _bounding_box.left()  - _time_offset,
                                             _bounding_box.right() - _time_offset } );
        }
    }

    const PlotData* transformedData() const { return _transformed_data; }

protected:
    QRectF _bounding_box;

private:
    const PlotData* _transformed_data;
    double _time_offset;
};

#endif // SERIES_DATA_H
