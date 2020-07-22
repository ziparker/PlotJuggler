#include "plotwidget_editor.h"
#include "ui_plotwidget_editor.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSettings>
#include <QListWidgetItem>
#include <QLabel>

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
};

PlotwidgetEditor::PlotwidgetEditor(PlotWidget *plotwidget, QWidget *parent) :
  QDialog(parent),
  ui(new Ui::plotwidget_editor),
  _plotwidget_origin(plotwidget)
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

void PlotwidgetEditor::on_buttonBox_accepted()
{
  this->accept();
  QDomDocument doc;
  auto elem = _plotwidget->xmlSaveState(doc);
  _plotwidget_origin->xmlLoadState( elem );
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


void PlotwidgetEditor::on_tableWidget_itemSelectionChanged()
{
//  QModelIndex index = ui->tableWidget->selectionModel()->selectedRows().first();
//  auto item = ui->tableWidget->item( index.row(), index.column());
//  QColor col = item->foreground().color();
//  _color_wheel->setColor(col);

}

void PlotwidgetEditor::on_tableWidget_cellClicked(int row, int column)
{
//  for(int i=0; i< ui->tableWidget->rowCount(); i++)
//  {
//    auto item = ui->tableWidget->item(i, 0);
//    if( i== row)
//    {
//      item->setSelected( !item->isSelected() );
//    }
//    else{
//      item->setSelected(false);
//    }
//  }

}

void PlotwidgetEditor::on_listWidget_currentRowChanged(int currentRow)
{
  auto item =  ui->listWidget->item(currentRow);
  auto row_widget = dynamic_cast<RowWidget*>(  ui->listWidget->itemWidget(item) );
  _color_wheel->setColor( row_widget->color() );
}
