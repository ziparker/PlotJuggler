#ifndef FIRST_DERIVATIVE_H
#define FIRST_DERIVATIVE_H

#include "PlotJuggler/transform_function.h"
#include <QLineEdit>

class FirstDerivative: public SeriesTransform
{
public:
  FirstDerivative();

  const char* name() const override { return "1st Derivative"; }

  void reset() override {}

  PlotData::Point calculatePoint(const PlotData& src_data, size_t point_index) override;

  QWidget* optionsWidget() override;

  bool xmlSaveState(QDomDocument& doc, QDomElement& parent_element) const override;

  bool xmlLoadState(const QDomElement& parent_element) override;

private:

  QLineEdit* _interval_linedit;
  double _interval;
};

#endif // FIRST_DERIVATIVE_H
