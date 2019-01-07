#ifndef CUSTOM_TIMESERIES_H
#define CUSTOM_TIMESERIES_H

#include "timeseries_qwt.h"
#include "custom_function.h"
#include "PlotJuggler/plotdata.h"

class CustomTimeseries: public TimeseriesQwt
{
public:
    CustomTimeseries(const PlotData *base,
                     const SnippetData &snippet,
                     PlotDataMapRef& mapped_data,
                     double time_offset);

    bool updateCache() override;

private:
    CustomFunction _transform;
    const PlotDataMapRef& _mapped_data;
};

#endif // CUSTOM_TIMESERIES_H
