#ifndef SCALE_TRANSFORM_H
#define SCALE_TRANSFORM_H

#include <QWidget>
#include "PlotJuggler/transform_function.h"

namespace Ui {
class ScaleTransform;
}

class ScaleTransform : public TimeSeriesTransform
{

public:
  explicit ScaleTransform();

  ~ScaleTransform() override;

  const char* name() const override;

  void calculate(PlotData* dst_data) override;

  QWidget* optionsWidget() override;

  bool xmlSaveState(QDomDocument& doc, QDomElement& parent_element) const override;

  bool xmlLoadState(const QDomElement& parent_element) override;

private:
  QWidget *_widget;
  Ui::ScaleTransform *ui;
};

#endif // SCALE_TRANSFORM_H
