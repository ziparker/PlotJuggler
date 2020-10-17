#include "outlier_removal.h"
#include "ui_outlier_removal.h"

OutlierRemovalFilter::OutlierRemovalFilter():
  ui(new Ui::OutlierRemovalFilter),
  _widget(new QWidget()),
  _buffer(5),
  _ring_view( _buffer.begin(), _buffer.end() )
{
  ui->setupUi(_widget);

  connect(ui->spinBoxFactor, qOverload<double>(&QDoubleSpinBox::valueChanged),
          this, [=](int){ emit parametersChanged();  } );
}

OutlierRemovalFilter::~OutlierRemovalFilter()
{
  delete ui;
  delete _widget;
}

void OutlierRemovalFilter::calculate(PlotData *dst_data)
{
  dst_data->clear();

  const size_t N = dataSource()->size();

  if ( N < 5 )
  {
    for(size_t i=0; i < N; i++)
    {
      const auto& p = dataSource()->at(i);
      dst_data->pushBack(p);
    }
  }
  else{

    // add the first 2 points
    dst_data->pushBack( dataSource()->at(0) );
    dst_data->pushBack( dataSource()->at(1) );

    // fill the beginning of the ring
    for(size_t i=0; i < 4; i++)
    {
      const auto& p = dataSource()->at(i);
      _ring_view.push_back(p.y);
    }

    for(size_t i=5; i < N; i++)
    {
      const auto& p = dataSource()->at(i);
      _ring_view.push_back(p.y);

      bool is_outlier = false;

      double d1 = (_ring_view[1] -  _ring_view[2]);
      double d2 = (_ring_view[2] -  _ring_view[3]);
      if( d1*d2 < 0 ) // spike
      {
        double left  = std::abs(_ring_view[0] -  _ring_view[1]);
        double right = std::abs(_ring_view[3] -  _ring_view[4]);
        double thresh = std::max( left, right ) * ui->spinBoxFactor->value();

        double jump = std::max(std::abs(d1), std::abs(d2));
        if( jump > thresh )
        {
          is_outlier = true;
        }
      }
      if( !is_outlier ){
        dst_data->pushBack( dataSource()->at(i-2) );
      }
    }
    // add the last 2 points
    dst_data->pushBack( dataSource()->at( N-2) );
    dst_data->pushBack( dataSource()->at( N-1) );
  }
}

QWidget *OutlierRemovalFilter::optionsWidget()
{
  return _widget;
}

bool OutlierRemovalFilter::xmlSaveState(QDomDocument &doc, QDomElement &parent_element) const
{
  QDomElement widget_el = doc.createElement("options");
  widget_el.setAttribute("factor",  ui->spinBoxFactor->value() );
  parent_element.appendChild( widget_el );
  return true;
}

bool OutlierRemovalFilter::xmlLoadState(const QDomElement &parent_element)
{
  QDomElement widget_el = parent_element.firstChildElement("options");
  ui->spinBoxFactor->setValue( widget_el.attribute("value", "100.0").toDouble() );
  return true;
}
