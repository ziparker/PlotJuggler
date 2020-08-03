#include "first_derivative.h"
#include <QFormLayout>
#include <QDoubleValidator>

FirstDerivative::FirstDerivative()
{
  _interval = 1.0;
}

PlotData::Point FirstDerivative::calculatePoint(const PlotData &src_data, size_t point_index)
{
  return  {};
}

QWidget *FirstDerivative::optionsWidget()
{
  auto widget = new QWidget();
  auto layout = new QFormLayout();
  widget->setLayout(layout);

  _interval_linedit = new QLineEdit();
  _interval_linedit->setValidator( new QDoubleValidator(0.0001, 1000, 4, _interval_linedit) );
  _interval_linedit->setText("1.0");

  connect( _interval_linedit, &QLineEdit::editingFinished, this,
          [this]()
          {
            _interval = _interval_linedit->text().toDouble();
            reset();
          });

  layout->addRow("Time interval", _interval_linedit);

  return widget;
}

bool FirstDerivative::xmlSaveState(QDomDocument &doc, QDomElement &parent_element) const
{
  return false;
}

bool FirstDerivative::xmlLoadState(const QDomElement &parent_element)
{
  return false;
}
