#include "plotwidget_editor.h"
#include "ui_plotwidget_editor.h"
#include <QVBoxLayout>
#include <QSettings>


PlotwidgetEditor::PlotwidgetEditor(PlotWidget *plotwidget, QWidget *parent) :
  QDialog(parent),
  ui(new Ui::plotwidget_editor)
{
  ui->setupUi(this);
  auto layout = new QVBoxLayout();

//  setWindowFlags(windowFlags() & ~Qt::WindowCloseButtonHint);


  QPalette pal = palette();
  pal.setColor(QPalette::Background, Qt::white);
  ui->plotContainer->setAutoFillBackground(true);
  ui->plotContainer->setPalette(pal);

  setupColorWidget();

  QDomDocument doc;

  auto saved_state = plotwidget->xmlSaveState(doc);

  _plotwidget = new PlotWidget(plotwidget->datamap());
  _plotwidget->xmlLoadState(saved_state);

  _plotwidget->on_changeTimeOffset( plotwidget->timeOffset() );

  ui->plotContainer->setLayout(layout);
  layout->addWidget(_plotwidget);

  _plotwidget->zoomOut(false);

  QSettings settings;
  restoreGeometry(settings.value("PlotwidgetEditor.geometry").toByteArray());

}

PlotwidgetEditor::~PlotwidgetEditor()
{
  QSettings settings;
  settings.setValue("PlotwidgetEditor.geometry", saveGeometry());

  delete ui;
}

void PlotwidgetEditor::onColorChanged(QColor c)
{
  ui->editColotText->setText( c.name() );
}

void PlotwidgetEditor::setupColorWidget()
{
  auto wheel_layout = new QVBoxLayout(this);
  wheel_layout->setMargin(0);
  wheel_layout->setSpacing(5);
  ui->widgetWheel->setLayout( wheel_layout );
  _color_wheel = new color_widgets::ColorWheel(this);
  wheel_layout->addWidget(_color_wheel);

  _color_preview = new color_widgets::ColorPreview(this);
  _color_preview->setMaximumHeight(25);

  wheel_layout->addWidget(_color_preview);

  connect(_color_wheel, &color_widgets::ColorWheel::colorChanged,
          this, &PlotwidgetEditor::onColorChanged);

  connect(_color_wheel, &color_widgets::ColorWheel::colorChanged,
          _color_preview, &color_widgets::ColorPreview::setColor );

  _color_wheel->setColor(Qt::blue);
}

void PlotwidgetEditor::on_editColotText_textChanged(const QString &text)
{
  if( QColor::isValidColor(text))
  {
    QColor col(text);
    _color_wheel->setColor( col );
    _color_preview->setColor( col );
  }
}

void PlotwidgetEditor::on_radioLines_toggled(bool checked)
{
  if(checked)
  {
    _plotwidget->changeCurveStyle( QwtPlotCurve::Lines );
  }
}


void PlotwidgetEditor::on_radioPoints_toggled(bool checked)
{
  if(checked)
  {
    _plotwidget->changeCurveStyle( QwtPlotCurve::Dots );
  }
}

void PlotwidgetEditor::on_radioBoth_toggled(bool checked)
{
  if(checked)
  {
    _plotwidget->changeCurveStyle( QwtPlotCurve::LinesAndDots );
  }
}

void PlotwidgetEditor::on_buttonBox_accepted()
{
  this->accept();
}

void PlotwidgetEditor::on_buttonBox_rejected()
{
  this->reject();
}

void PlotwidgetEditor::on_checkBoxMax_toggled(bool checked)
{
  ui->lineLimitMax->setEnabled(checked);
}

void PlotwidgetEditor::on_checkBoxMin_toggled(bool checked)
{
  ui->lineLimitMin->setEnabled(checked);
}

