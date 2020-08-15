#ifndef PLOTDATA_QWT_H
#define PLOTDATA_QWT_H

#include "series_data.h"
#include "PlotJuggler/plotdata.h"
#include "PlotJuggler/transform_function.h"

class TimeSeries : public DataSeriesBase
{
public:
  TimeSeries(const PlotData* source_data);

  PlotData::RangeValueOpt getVisualizationRangeY(PlotData::RangeTime range_X) override;

  nonstd::optional<QPointF> sampleFromTime(double t) override;

  TimeSeriesTransformPtr transform();

  void setTransform(QString transform_ID);

  virtual bool updateCache() override;

  QString transformName();

protected:
  const PlotData* _source_data;
  PlotData _dst_data;
  TimeSeriesTransformPtr _transform;
};

//---------------------------------------------------------



#endif  // PLOTDATA_H
