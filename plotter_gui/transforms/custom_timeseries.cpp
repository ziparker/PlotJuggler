#include "custom_timeseries.h"
#include <QSettings>
#include "lua_custom_function.h"
#include "qml_custom_function.h"

CustomTimeseries::CustomTimeseries(const PlotData *source_data,
                                   const SnippetData &snippet,
                                   PlotDataMapRef &mapped_data):
    TimeseriesQwt( source_data, &_cached_data ),
    _mapped_data(mapped_data)
{
    _transform = CustomFunctionFactory(source_data->name(), snippet);
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

    _transform->calculate( _mapped_data, &_cached_data );
    calculateBoundingBox();

    return true;
}
