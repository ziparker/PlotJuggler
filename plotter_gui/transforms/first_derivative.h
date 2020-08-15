#ifndef FIRST_DERIVATIVE_H
#define FIRST_DERIVATIVE_H

#include <QLineEdit>
#include "PlotJuggler/transform_function.h"
#include "ui_first_derivative.h"


class FirstDerivative: public TimeSeriesTransform
{
public:
  FirstDerivative();

  ~FirstDerivative() override;

  const char* name() const override { return "1st Derivative"; }

  void reset() override {}

  void calculate(PlotData* dst_data) override;

  QWidget* optionsWidget() override;

  bool xmlSaveState(QDomDocument& doc, QDomElement& parent_element) const override;

  bool xmlLoadState(const QDomElement& parent_element) override;

private slots:

  void on_buttonCompute_clicked();

private:

  QWidget *_widget;
  Ui::FirstDerivariveForm* ui;
  double _dT;
};

#endif // FIRST_DERIVATIVE_H
