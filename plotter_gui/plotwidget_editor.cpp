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
#include <QDebug>

const double MAX_DOUBLE = std::numeric_limits<double>::max() / 2;

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

  _bounding_rect = _plotwidget_origin->canvasBoundingRect();

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
  auto range_x = _plotwidget->getMaximumRangeX();
  PlotData::RangeValue suggested_limits = _plotwidget->getMaximumRangeY(range_x);

  if( ylimits.min != -MAX_DOUBLE)
  {
    ui->checkBoxMin->setChecked(true);
    ui->lineLimitMin->setText(QString::number(ylimits.min));
  }
  else{
    ui->lineLimitMin->setText(QString::number(suggested_limits.min));
  }

  if( ylimits.max != MAX_DOUBLE)
  {
    ui->checkBoxMax->setChecked(true);
    ui->lineLimitMax->setText(QString::number(ylimits.max));
  }
  else{
    ui->lineLimitMax->setText(QString::number(suggested_limits.max));
  }

  ui->listWidget->setStyleSheet("QListView::item:selected { background: #ddeeff; }");

  auto names = TransformFactory::registeredNames();
  for(const auto& name: names)
  {
    ui->comboTransform->addItem( QString::fromStdString(name) );
  }

  if( ui->listWidget->count() != 0 ){
    ui->listWidget->item(0)->setSelected(true);
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
  auto selected = ui->listWidget->selectedItems();
  if( selected.size() != 1)
  {
    return;
  }
  auto item =  selected.front();
  if( item ){
    auto row_widget = dynamic_cast<RowWidget*>(  ui->listWidget->itemWidget(item) );
    auto name = row_widget->text();
    if( row_widget->color() != c)
    {
      row_widget->setColor( c );
    }

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

void PlotwidgetEditor::onDeleteRow(QWidget* w)
{
  int row_count = ui->listWidget->count();
  for(int row = 0; row < row_count; row++ )
  {
    auto item = ui->listWidget->item(row);
    auto widget = ui->listWidget->itemWidget(item);
    if( widget == w )
    {
      QString curve = dynamic_cast<RowWidget*>(w)->text();
      qDebug() << "delete " << curve;
      ui->listWidget->takeItem(row);
      _plotwidget->removeCurve(curve.toStdString());
      widget->deleteLater();
      row_count--;
      break;
    }
  }
  if( row_count == 0)
  {
    disableWidgets();
  }
  _plotwidget->replot();
}

void PlotwidgetEditor::disableWidgets()
{
  ui->widgetColor->setEnabled(false);
  _color_wheel->setEnabled(false);
  _color_preview->setEnabled(false);

  ui->groupBoxLimits->setEnabled(false);
  ui->groupBoxStyle->setEnabled(false);

  ui->comboTransform->setEnabled(false);
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
    auto plot_row = new RowWidget(text, color) ;
    item->setSizeHint( plot_row->sizeHint() );
    ui->listWidget->setItemWidget(item, plot_row );

    connect(plot_row, &RowWidget::deleteRow, this, [this](QWidget* w) { onDeleteRow(w); });
    row++;
  }
  if( row == 0 )
  {
    disableWidgets();
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
  QDomDocument doc;
  _plotwidget->setZoomRectangle(_bounding_rect, false);
  auto elem = _plotwidget->xmlSaveState(doc);
  _plotwidget_origin->xmlLoadState( elem );
  this->accept();
}

RowWidget::RowWidget(QString text, QColor color): QWidget()
{
  setMouseTracking(true);
  const QSize button_size(20,20);

  auto layout = new QHBoxLayout();
  setLayout( layout );
  _text = new QLabel(text, this);

  _empty_spacer = new QWidget();
  _empty_spacer->setFixedSize( button_size );

  setColor(color);
  _delete_button = new QPushButton(this);
  _delete_button->setFlat(true);
  _delete_button->setFixedSize( button_size );
  auto icon = QIcon(":/resources/svg/remove_red.svg");
  _delete_button->setStyleSheet("QPushButton:hover{ border: 0px;}");

  _delete_button->setIcon( icon) ;
  _delete_button->setIconSize( button_size );

  layout->addWidget(_empty_spacer);
  layout->addWidget(_delete_button);
  layout->addWidget(_text);

  _delete_button->setHidden(true);

  connect( _delete_button, &QPushButton::clicked, this, [this](){ emit deleteRow(this); });
}

void RowWidget::enterEvent(QEvent *ev)
{
  _delete_button->setHidden(false);
  _empty_spacer->setHidden(true);
}

void RowWidget::leaveEvent(QEvent *ev)
{
  _delete_button->setHidden(true);
  _empty_spacer->setHidden(false);
}

QString RowWidget::text() const
{
  return _text->text();
}

void RowWidget::setColor(QColor color)
{
  setStyleSheet( QString("color: %1;").arg(color.name()));
  _color = color;
}

QColor RowWidget::color() const
{
  return _color;
}

void PlotwidgetEditor::on_listWidget_itemSelectionChanged()
{
  auto selected = ui->listWidget->selectedItems();
  if( selected.size() == 0 || ui->listWidget->count() == 0)
  {
    ui->widgetColor->setEnabled(false);
    ui->comboTransform->setEnabled(false);
    ui->stackedWidgetArguments->setEnabled(false);
    ui->editColotText->setText( "#000000" );
    return;
  }

  ui->widgetColor->setEnabled(true);
  ui->stackedWidgetArguments->setEnabled(true);

  if( selected.size() > 1)
  {
    return;
  }

  auto item = selected.front();
  auto row_widget = dynamic_cast<RowWidget*>(  ui->listWidget->itemWidget(item) );
  if( row_widget )
  {
    ui->editColotText->setText( row_widget->color().name() );
  }
  if( _plotwidget->isXYPlot() == false )
  {
    ui->comboTransform->setEnabled(true);
    auto curve_name = row_widget->text();
    QwtPlotCurve* qwt_curve = _plotwidget->curveList().at(curve_name.toStdString());
    TimeSeries* ts = dynamic_cast<TimeSeries*>( qwt_curve->data() );
    if( !ts->transform() )
    {
      ui->comboTransform->setCurrentIndex(0);
    }
    else{
      ui->comboTransform->setCurrentText( ts->transformName() );
    }
  }

  on_comboTransform_currentIndexChanged( ui->comboTransform->currentText() );
}

void PlotwidgetEditor::on_comboTransform_currentIndexChanged(const QString &transform_ID)
{
  auto selected = ui->listWidget->selectedItems();
  auto item = selected.front();
  auto curve_name = dynamic_cast<RowWidget*>( ui->listWidget->itemWidget(item) )->text();

  QwtPlotCurve* qwt_curve = _plotwidget->curveList().at(curve_name.toStdString());
  TimeSeries* ts = dynamic_cast<TimeSeries*>( qwt_curve->data() );

  if( transform_ID == ui->comboTransform->itemText(0) )
  {
    ts->setTransform({});
    ui->stackedWidgetArguments->setCurrentIndex(0);
    qDebug() << "stackedWidgetArguments " << 0;
  }
  else{
    ts->setTransform(transform_ID);
    auto widget = ts->transform()->optionsWidget();
    int index = ui->stackedWidgetArguments->indexOf(widget);
    if( index == -1)
    {
      index = ui->stackedWidgetArguments->addWidget(widget);
    }

    ui->stackedWidgetArguments->setCurrentIndex(index);
    qDebug() << "stackedWidgetArguments " << index;

    if( _connected_transform_widgets.count(widget) == 0)
    {
      connect( ts->transform().get(), &TimeSeriesTransform::parametersChanged,
                this, [=]() {
                  ts->updateCache();
                  _plotwidget->zoomOut(false);
                });
      _connected_transform_widgets.insert(widget);
    }
  }

  _plotwidget->zoomOut(false);
}
