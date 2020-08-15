#include "first_derivative.h"
#include <QFormLayout>
#include <QDoubleValidator>

FirstDerivative::FirstDerivative():
  _widget(nullptr),
  _dT(0.0)
{
}

void FirstDerivative::calculate(PlotData *dst_data)
{
  dst_data->clear();

  if(!dataSource() || dataSource()->size() < 2)
  {
    return;
  }

  double prev_x = dataSource()->front().x;
  double prev_y = dataSource()->front().y;

  for(size_t i=1; dataSource()->size(); i++)
  {
    const auto& p = dataSource()->at(i);

    double dt = (_dT == 0.0) ? (p.x - prev_x) : _dT;

    double der = (p.y - prev_y) / dt;
    dst_data->pushBack({prev_x, der});
    prev_x = p.x;
    prev_y = p.y;
  }

  dst_data->pushBack( {prev_x, prev_y} );
}


QWidget *FirstDerivative::optionsWidget()
{
  if(!_widget)
  {
    _widget = new QWidget;
    ui->setupUi(_widget);
    ui->lineEditCustom->setValidator( new QDoubleValidator(0.0001, 1000, 4, ui->lineEditCustom) );
  }
  const size_t data_size = dataSource()->size();

  if(!dataSource() || data_size < 2)
  {
    _widget->setEnabled(false);
  }
  return _widget;
}

bool FirstDerivative::xmlSaveState(QDomDocument &doc, QDomElement &parent_element) const
{
  return false;
}

bool FirstDerivative::xmlLoadState(const QDomElement &parent_element)
{
  return false;
}

void FirstDerivative::on_buttonCompute_clicked()
{
  if(!dataSource() || dataSource()->size() < 2)
  {
    return;
  }

  const size_t data_size = dataSource()->size();

  // calculate automatic diff
  std::vector<double> diff;
  diff.reserve(data_size-1);
  double prev_t = dataSource()->at(0).x;
  for(size_t i=1; i<data_size; i++)
  {
    double t = dataSource()->at(i).x;
    double delta = t - prev_t;
    prev_t = t;
    diff.push_back(delta);
  }

  size_t first = 0;
  size_t last = diff.size();
  if( data_size > 10 )
  {
    std::sort(diff.begin(), diff.end());
    first = last / 5;
    last = (last*4)/5;
  }
  double total = 0;
  for(size_t i=first; i<last; i++) {
    total += diff[i];
  }
  double estimated_dt = total / static_cast<double>(last-first);
  ui->lineEditCustom->setText(QString::number(estimated_dt, 'g', 4));

  if( std::abs(estimated_dt - _dT) > std::numeric_limits<double>::epsilon() )
  {
    emit parametersChanged();
  }
}
