#ifndef CUSTOM_TIMESERIES_H
#define CUSTOM_TIMESERIES_H

#include "timeseries_qwt.h"
#include "custom_transform.h"
#include "PlotJuggler/plotdata.h"

class CustomTimeseries: public TimeseriesQwt
{
public:
    CustomTimeseries(const PlotData* base,
                     const QString &global_vars,
                     const QString &function,
                     PlotDataMapRef& mapped_data,
                     double time_offset);

    bool updateCache() override;

private:
    CustomTransform _transform;
    const PlotDataMapRef& _mapped_data;
};

#endif // CUSTOM_TIMESERIES_H
