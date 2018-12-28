#ifndef PLOTDATA_QWT_H
#define PLOTDATA_QWT_H

#include "series_data.h"
#include "PlotJuggler/plotdata.h"

class TimeseriesQwt: public DataSeriesBase
{
public:

    TimeseriesQwt(const PlotData* base, double time_offset);

    PlotData::RangeTimeOpt getVisualizationRangeX();

    PlotData::RangeValueOpt getVisualizationRangeY(PlotData::RangeTime range_X) override;

    nonstd::optional<QPointF> sampleFromTime(double t) override;

public slots:

    virtual void onTimeoffsetChanged(double offset) override;

protected:

    int getIndexFromTime(double t);

    const PlotData* _plot_data;
};

//---------------------------------------------------------

class Timeseries_NoTransform: public TimeseriesQwt
{
public:
    Timeseries_NoTransform(const PlotData* base, double time_offset):
        TimeseriesQwt(base,time_offset)
    {
        updateCache();
    }

     bool updateCache() override;
};

class Timeseries_1stDerivative: public TimeseriesQwt
{
public:
    Timeseries_1stDerivative(const PlotData* base, double time_offset):
        TimeseriesQwt(base,time_offset)
    {
        updateCache();
    }

     bool updateCache() override;
};

class Timeseries_2ndDerivative: public TimeseriesQwt
{
public:
    Timeseries_2ndDerivative(const PlotData* base, double time_offset):
        TimeseriesQwt(base,time_offset)
    {
        updateCache();
    }

     bool updateCache() override;
};



#endif // PLOTDATA_H
