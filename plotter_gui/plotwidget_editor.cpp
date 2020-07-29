#include "plotwidget_editor.h"
#include "ui_plotwidget_editor.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSettings>
#include <QListWidgetItem>
#include <QLabel>
#include <QMouseEvent>
#include <QPushButton>
#include <QDialogButtonBox>

const double MAX_DOUBLE = std::numeric_limits<double>::max() / 2;

class RowWidget: public QWidget
{
public:
  RowWidget(QString text, QColor color): QWidget()
  {
    auto layout = new QHBoxLayout();
    setLayout( layout );
    _text = new QLabel(text, this);
    layout->addWidget(_text);

    setColor(color);
    _delete_button = new QPushButton(this);
    _delete_button->setFlat(true);
    _delete_button->setFixedSize( QSize(20,20) );
  }

  void mouseMoveEvent(QMouseEvent *ev) override
  {
    if (this->rect().contains(ev->pos())) {
      // Mouse over Widget
    }
  }

  QString text() const
  {
    return _text->text();
  }

  void setColor(QColor color)
  {
    setStyleSheet( QString("color: %1;").arg(color.name()));
    _color = color;
  }

  QColor color() const
  {
    return _color;
  }

private:
  QLabel* _text;
  QColor _color;
  QPushButton* _delete_button;
};

PlotwidgetEditor::PlotwidgetEditor(PlotWidget *plotwidget, QWidget *parent) :
  QDialog(parent),
  ui(new Ui::plotwidget_editor),
  _plotwidget_origin(plotwidget)
{
  ui->setupUi(this);
  auto layout = new QVBoxLayout();

  installEventFilter(this);

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

  setupTable();

  QSettings settings;
  restoreGeometry(settings.value("PlotwidgetEditor.geometry").toByteArray());

  if( _plotwidget->curveStyle() == QwtPlotCurve::Lines)
  {
    ui->radioLines->setChecked(true);
  }
  else if( _plotwidget->curveStyle() == QwtPlotCurve::Dots)
  {
    ui->radioPoints->setChecked(true);
  }
  else {
    ui->radioBoth->setChecked(true);
  }

  ui->lineLimitMax->setValidator(new QDoubleValidator(this));
  ui->lineLimitMin->setValidator(new QDoubleValidator(this));

  auto ylimits = _plotwidget->customAxisLimit();
  if( ylimits.min != -MAX_DOUBLE)
  {
    ui->checkBoxMin->setChecked(true);
    ui->lineLimitMin->setText(QString::number(ylimits.min));
  }

  if( ylimits.max != MAX_DOUBLE)
  {
    ui->checkBoxMax->setChecked(true);
    ui->lineLimitMax->setText(QString::number(ylimits.max));
  }
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

  auto item =  ui->listWidget->currentItem();
  if( item ){
    auto row_widget = dynamic_cast<RowWidget*>(  ui->listWidget->itemWidget(item) );
    row_widget->setColor( c );

    _plotwidget->on_changeCurveColor( row_widget->text().toStdString(), c );
  }
}

void PlotwidgetEditor::setupColorWidget()
{
  auto wheel_layout = new QVBoxLayout();
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

void PlotwidgetEditor::setupTable()
{
  std::map<std::string, QColor> colors = _plotwidget->getCurveColors();

  int row = 0;
  for (auto& it : colors)
  {
    auto text = QString::fromStdString(it.first);
    auto color = it.second;
    auto item = new QListWidgetItem();
  //  item->setForeground(color);
    ui->listWidget->addItem( item );
    auto button = new RowWidget(text, color) ;
    item->setSizeHint( button->sizeHint() );
    ui->listWidget->setItemWidget(item, button );

    row++;
  }
}

void PlotwidgetEditor::updateLimits()
{
  double ymin = -MAX_DOUBLE;
  double ymax = MAX_DOUBLE;

  if (ui->checkBoxMax->isChecked() && !ui->lineLimitMax->text().isEmpty())
  {
    bool ok = false;
    double val = ui->lineLimitMax->text().toDouble(&ok);
    if(ok) { ymax = val; }
  }

  if (ui->checkBoxMin->isChecked() && !ui->lineLimitMin->text().isEmpty())
  {
    bool ok = false;
    double val = ui->lineLimitMin->text().toDouble(&ok);
    if(ok) { ymin = val; }
  }

  if (ymin > ymax)
  {
    // swap
    ui->lineLimitMin->setText(QString::number(ymax));
    ui->lineLimitMax->setText(QString::number(ymin));
    std::swap(ymin, ymax);
  }

  PlotData::RangeValue range;
  range.min = ymin;
  range.max = ymax;
  _plotwidget->setCustomAxisLimits(range);
}

void PlotwidgetEditor::on_editColotText_textChanged(const QString &text)
{
  if( text.size() == 7 && text[0] == '#' && QColor::isValidColor(text))
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

void PlotwidgetEditor::on_checkBoxMax_toggled(bool checked)
{
  ui->lineLimitMax->setEnabled(checked);
  updateLimits();
}

void PlotwidgetEditor::on_checkBoxMin_toggled(bool checked)
{
  ui->lineLimitMin->setEnabled(checked);
  updateLimits();
}

void PlotwidgetEditor::on_listWidget_currentRowChanged(int currentRow)
{
  auto item =  ui->listWidget->item(currentRow);
  auto row_widget = dynamic_cast<RowWidget*>(  ui->listWidget->itemWidget(item) );
  _color_wheel->setColor( row_widget->color() );
}

void PlotwidgetEditor::on_pushButtonReset_clicked()
{
  PlotData::RangeValue no_limits;
  no_limits.min = -MAX_DOUBLE;
  no_limits.max = +MAX_DOUBLE;

  _plotwidget->setCustomAxisLimits(no_limits);

  auto range_x = _plotwidget->getMaximumRangeX();
  PlotData::RangeValue limits = _plotwidget->getMaximumRangeY(range_x);

  ui->lineLimitMin->setText(QString::number(limits.min));
  ui->lineLimitMax->setText(QString::number(limits.max));
}

void PlotwidgetEditor::on_lineLimitMax_editingFinished()
{
   updateLimits();
}

void PlotwidgetEditor::on_lineLimitMin_editingFinished()
{
   updateLimits();
}

void PlotwidgetEditor::on_pushButtonCancel_pressed()
{
  this->reject();
}

void PlotwidgetEditor::on_pushButtonSave_pressed()
{
  this->accept();
  QDomDocument doc;
  auto elem = _plotwidget->xmlSaveState(doc);
  _plotwidget_origin->xmlLoadState( elem );
}
