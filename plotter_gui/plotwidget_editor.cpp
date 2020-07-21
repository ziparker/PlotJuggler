#include "plotwidget_editor.h"
#include "ui_plotwidget_editor.h"
#include <QHBoxLayout>
#include <QSettings>

PlotwidgetEditor::PlotwidgetEditor(PlotWidget *plotwidget, QWidget *parent) :
  QDialog(parent),
  ui(new Ui::plotwidget_editor)
{
  ui->setupUi(this);
  auto layout = new QHBoxLayout();

  QPalette pal = palette();
  pal.setColor(QPalette::Background, Qt::white);
  ui->plotContainer->setAutoFillBackground(true);
  ui->plotContainer->setPalette(pal);

  QSettings settings;
  restoreGeometry(settings.value("PlotwidgetEditor.geometry").toByteArray());

  QDomDocument doc;

  auto saved_state = plotwidget->xmlSaveState(doc);

  PlotWidget* copied_plotwidget = new PlotWidget(plotwidget->datamap());
  copied_plotwidget->xmlLoadState(saved_state);

  copied_plotwidget->on_changeTimeOffset( plotwidget->timeOffset() );

  ui->plotContainer->setLayout(layout);
  layout->addWidget(copied_plotwidget);

  copied_plotwidget->zoomOut(false);
}

PlotwidgetEditor::~PlotwidgetEditor()
{
  QSettings settings;
  settings.setValue("PlotwidgetEditor.geometry", saveGeometry());

  delete ui;
}
