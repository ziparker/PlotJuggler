#include "timeseries_qwt.h"
#include <limits>
#include <stdexcept>
#include <QMessageBox>
#include <QPushButton>
#include <QString>

TimeSeries::TimeSeries(const PlotData* source_data)
  : DataSeriesBase(source_data),
  _source_data(source_data),
  _dst_data(source_data->name())
{
}

PlotData::RangeValueOpt TimeSeries::getVisualizationRangeY(PlotData::RangeTime range_X)
{
  int first_index = plotData()->getIndexFromX(range_X.min);
  int last_index = plotData()->getIndexFromX(range_X.max);

  if (first_index > last_index || first_index < 0 || last_index < 0)
  {
    return PlotData::RangeValueOpt();
  }

  if (first_index == 0 && last_index == plotData()->size() - 1)
  {
    return PlotData::RangeValueOpt({ _bounding_box.bottom(), _bounding_box.top() });
  }

  double min_y = (std::numeric_limits<double>::max());
  double max_y = (-std::numeric_limits<double>::max());

  for (size_t i = first_index; i < last_index; i++)
  {
    const double Y = sample(i).y();
    min_y = std::min(min_y, Y);
    max_y = std::max(max_y, Y);
  }
  return PlotData::RangeValueOpt({ min_y, max_y });
}

nonstd::optional<QPointF> TimeSeries::sampleFromTime(double t)
{
  int index = plotData()->getIndexFromX(t);
  if (index < 0)
  {
    return nonstd::optional<QPointF>();
  }
  const auto& p = plotData()->at(size_t(index));
  return QPointF(p.x, p.y);
}

TimeSeriesTransformPtr TimeSeries::transform()
{
  return _transform;
}

bool TimeSeries::updateCache()
{
  if( _transform )
  {
    _transform->calculate( &_dst_data );
  }
  else{
    // TODO: optimize ??
    _dst_data.clear();
    for(size_t i=0; i < _source_data->size(); i++)
    {
      _dst_data.pushBack( _source_data->at(i) );
    }
  }
  calculateBoundingBox();
  return true;
}

QString TimeSeries::transformName()
{
  return ( !_transform ) ? QString() : _transform->name();
}

