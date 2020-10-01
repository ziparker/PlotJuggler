#include "moving_average_filter.h"
#include "ui_moving_average_filter.h"
#include <numeric>

MovingAverageFilter::MovingAverageFilter() :
  ui(new Ui::MovingAverageFilter),
  _widget(new QWidget()),
  _ring_view( _buffer.begin(), _buffer.end() )
{
  ui->setupUi(_widget);

  connect(ui->spinBoxSamples, qOverload<int>(&QSpinBox::valueChanged),
          this, [=](int){ emit parametersChanged();  } );
}

MovingAverageFilter::~MovingAverageFilter()
{
  delete ui;
  delete _widget;
}

void MovingAverageFilter::calculate(PlotData *dst_data)
{
  dst_data->clear();

  if ( dataSource()->size() == 0 )
  {
    return;
  }

  _buffer.clear();
  _buffer.resize( std::min( int(ui->spinBoxSamples->value()), int(dataSource()->size())) );
  _ring_view = nonstd::ring_span<double>(_buffer.begin(), _buffer.end());

  const auto p1 = dataSource()->at(0);

  for(size_t i=0; i < _buffer.size(); i++)
  {
    _ring_view.push_back(p1.y);
  }
  double total = p1.y * _ring_view.size();

  dst_data->pushBack( p1 );
  for(size_t i=1; i < dataSource()->size(); i++)
  {
    const auto& p = dataSource()->at(i);

    total -= _ring_view.front();
    _ring_view.pop_front();
    _ring_view.push_back( p.y );
    total += _ring_view.back();

    dst_data->pushBack({p.x, total / _ring_view.size() });
  }
}

QWidget *MovingAverageFilter::optionsWidget()
{
  return _widget;
}

bool MovingAverageFilter::xmlSaveState(QDomDocument &doc, QDomElement &parent_element) const
{
  QDomElement widget_el = doc.createElement("options");

  if( ui->radioSamples->isChecked() )
  {
    widget_el.setAttribute("radio", "radioSamples");
    widget_el.setAttribute("value", ui->spinBoxSamples->value());
  }
  else{
    widget_el.setAttribute("radio", "radioTime");
    widget_el.setAttribute("value", ui->spinBoxTime->value());
  }

  parent_element.appendChild( widget_el );
  return true;
}

bool MovingAverageFilter::xmlLoadState(const QDomElement &parent_element)
{
  QDomElement widget_el = parent_element.firstChildElement("options");

  if( widget_el.attribute("radio") == "radioSamples")
  {
    ui->radioSamples->setChecked(true);
    ui->spinBoxSamples->setValue( widget_el.attribute("value").toInt() );
  }
  else{
    ui->radioTime->setChecked(true);
    ui->spinBoxTime->setValue( widget_el.attribute("value").toDouble() );
  }
  return true;
}
