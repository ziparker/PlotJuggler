#include "custom_timeseries.h"


CustomTimeseries::CustomTimeseries(const PlotData *base,
                                   const SnippetData &snippet,
                                   PlotDataMapRef &mapped_data,
                                   double time_offset):
    TimeseriesQwt( base, time_offset ),
    _transform(base->name(), snippet),
    _mapped_data(mapped_data)
{
    updateCache();
}

bool CustomTimeseries::updateCache()
{
    if(_plot_data->size() == 0)
    {
        _cached_curve.clear();
        _bounding_box = QRectF();
        return true;
    }

    _transform.calculate( _mapped_data, &_cached_curve );

    double min_y =( std::numeric_limits<double>::max() );
    double max_y =(-std::numeric_limits<double>::max() );

    for ( auto& p: _cached_curve )
    {
        min_y = std::min( min_y, p.y );
        max_y = std::max( max_y, p.y );
        p.x -= _time_offset;
    }

    _bounding_box.setLeft(  _cached_curve.front().x );
    _bounding_box.setRight( _cached_curve.back().x );
    _bounding_box.setBottom( min_y );
    _bounding_box.setTop( max_y );
    return true;
}
