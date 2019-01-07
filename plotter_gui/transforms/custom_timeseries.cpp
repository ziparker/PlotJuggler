#include "custom_timeseries.h"


CustomTimeseries::CustomTimeseries(const PlotData *source_data,
                                   const SnippetData &snippet,
                                   PlotDataMapRef &mapped_data):
    TimeseriesQwt( source_data, &_cached_data ),
    _transform(source_data->name(), snippet),
    _mapped_data(mapped_data)
{
    updateCache();
}

bool CustomTimeseries::updateCache()
{
    if(_source_data->size() == 0)
    {
        _cached_data.clear();
        _bounding_box = QRectF();
        return true;
    }

    _transform.calculate( _mapped_data, &_cached_data );

    double min_y =( std::numeric_limits<double>::max() );
    double max_y =(-std::numeric_limits<double>::max() );

    for (const auto& p: _cached_data )
    {
        min_y = std::min( min_y, p.y );
        max_y = std::max( max_y, p.y );
    }

    _bounding_box.setLeft(  _cached_data.front().x );
    _bounding_box.setRight( _cached_data.back().x );
    _bounding_box.setBottom( min_y );
    _bounding_box.setTop( max_y );
    return true;
}
