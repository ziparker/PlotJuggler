#include "scale_transform.h"
#include "ui_scale_transform.h"

ScaleTransform::ScaleTransform() :
  _widget(new QWidget()),
  ui(new Ui::ScaleTransform)
{
  ui->setupUi(_widget);

//  ui->lineEditValue->setValidator( new QDoubleValidator() );

  connect(ui->buttonDegRad, &QPushButton::clicked,
          this, [=](){
            const double deg_rad = 3.14159265359 / 180;
            ui->lineEditValue->setText( QString::number(deg_rad, 'g', 5));
            emit parametersChanged();
          } );

  connect(ui->buttonRadDeg, &QPushButton::clicked,
          this, [=](){
            const double rad_deg = 180.0 / 3.14159265359;
            ui->lineEditValue->setText( QString::number(rad_deg, 'g', 5));
            emit parametersChanged();
          } );

  connect(ui->lineEditValue, &QLineEdit::editingFinished,
          this, [=](){
            emit parametersChanged();
          } );
}

ScaleTransform::~ScaleTransform()
{
  delete ui;
  delete _widget;
}

const char *ScaleTransform::name() const {
  return "Scale";
}

void ScaleTransform::calculate(PlotData *dst_data)
{
  dst_data->clear();
  double scale = ui->lineEditValue->text().toDouble();

  for(size_t i=0; i < dataSource()->size(); i++)
  {
    const auto& p = dataSource()->at(i);
    dst_data->pushBack({p.x, scale * p.y});
  }
}

QWidget *ScaleTransform::optionsWidget()
{
  return _widget;
}

bool ScaleTransform::xmlSaveState(QDomDocument &doc, QDomElement &parent_element) const
{
  QDomElement widget_el = doc.createElement("options");
  widget_el.setAttribute("value", ui->lineEditValue->text() );
  parent_element.appendChild( widget_el );

  return true;
}

bool ScaleTransform::xmlLoadState(const QDomElement &parent_element)
{
  QDomElement widget_el = parent_element.firstChildElement("options");
  ui->lineEditValue->setText( widget_el.attribute("value") );

  return true;
}
