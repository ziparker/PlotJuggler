#include "function_editor.h"
#include "custom_function.h"
#include "plotwidget.h"
#include <QDebug>
#include <QMessageBox>
#include <QFont>
#include <QDomDocument>
#include <QDomElement>
#include <QFontDatabase>
#include <QFile>
#include <QMenu>
#include <QAction>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QSettings>
#include <QByteArray>
#include <QInputDialog>
#include <QDragEnterEvent>
#include <QMimeData>
#include <QTableWidgetItem>

#include "lua_custom_function.h"
#include "svg_util.h"
#include "qml_custom_function.h"

FunctionEditorWidget::FunctionEditorWidget(PlotDataMapRef& plotMapData, const CustomPlotMap& mapped_custom_plots,
                                         QWidget* parent)
  : QWidget(parent)
  , _plot_map_data(plotMapData)
  , _custom_plots(mapped_custom_plots)
  , ui(new Ui::FunctionEditor)
  , _v_count(1)
{
  ui->setupUi(this);

  QSettings settings;

  this->setWindowTitle("Create a custom timeseries");

  const QFont fixedFont = QFontDatabase::systemFont(QFontDatabase::FixedFont);
  ui->globalVarsTextField->setFont(fixedFont);
  ui->mathEquation->setFont(fixedFont);
  ui->snippetTextEdit->setFont(fixedFont);
  ui->pushButtonDeleteCurves->setIcon(LoadSvgIcon(":/resources/svg/remove_red.svg", "light"));

  QPalette palette = ui->listAdditionalSources->palette();
  palette.setBrush(QPalette::Highlight, palette.brush(QPalette::Base));
  palette.setBrush(QPalette::HighlightedText, palette.brush(QPalette::Text));
  ui->listAdditionalSources->setPalette(palette);

  QStringList numericPlotNames;
  for (const auto& p : _plot_map_data.numeric)
  {
    QString name = QString::fromStdString(p.first);
    numericPlotNames.push_back(name);
  }
  numericPlotNames.sort(Qt::CaseInsensitive);

  QByteArray saved_xml = settings.value("AddCustomPlotDialog.recentSnippetsXML", QByteArray()).toByteArray();
  restoreGeometry(settings.value("AddCustomPlotDialog.geometry").toByteArray());

  if (saved_xml.isEmpty())
  {
    QFile file("://resources/default.snippets.xml");
    if (!file.open(QIODevice::ReadOnly))
    {
      throw std::runtime_error("problem with default.snippets.xml");
    }
    saved_xml = file.readAll();
  }

  importSnippets(saved_xml);

  ui->snippetsListSaved->setContextMenuPolicy(Qt::CustomContextMenu);

  connect(ui->snippetsListSaved, &QListWidget::customContextMenuRequested, this,
          &FunctionEditorWidget::savedContextMenu);

  ui->splitter->setStretchFactor(0, 3);
  ui->splitter->setStretchFactor(1, 2);

  ui->globalVarsTextField->setPlainText(settings.value("AddCustomPlotDialog.previousGlobals", "").toString());

  ui->mathEquation->setPlainText(settings.value("AddCustomPlotDialog.previousFunction", "return value").toString());

  ui->lineEditSource->installEventFilter(this);
  ui->listAdditionalSources->installEventFilter(this);
}

FunctionEditorWidget::~FunctionEditorWidget()
{
  QSettings settings;
  settings.setValue("AddCustomPlotDialog.recentSnippetsXML", exportSnippets());
  settings.setValue("AddCustomPlotDialog.geometry", saveGeometry());
  settings.setValue("AddCustomPlotDialog.previousGlobals", ui->globalVarsTextField->toPlainText());
  settings.setValue("AddCustomPlotDialog.previousFunction", ui->mathEquation->toPlainText());

  delete ui;
}

void FunctionEditorWidget::setLinkedPlotName(const QString& linkedPlotName)
{
  ui->lineEditSource->setText(linkedPlotName);
}

void FunctionEditorWidget::clear()
{
  ui->lineEditSource->setText("");
  ui->nameLineEdit->setText("");
  ui->listAdditionalSources->setRowCount(0);
}

void FunctionEditorWidget::setEditorMode(EditorMode mode)
{
  ui->label_linkeChannel->setVisible(mode != FUNCTION_ONLY);
  ui->pushButtonCreate->setVisible(mode != FUNCTION_ONLY);
  ui->pushButtonSave->setVisible(mode != TIMESERIES_ONLY);
}

QString FunctionEditorWidget::getLinkedData() const
{
  return ui->lineEditSource->text();
}

QString FunctionEditorWidget::getGlobalVars() const
{
  return ui->globalVarsTextField->toPlainText();
}

QString FunctionEditorWidget::getEquation() const
{
  return ui->mathEquation->toPlainText();
}

QString FunctionEditorWidget::getName() const
{
  return ui->nameLineEdit->text();
}


void FunctionEditorWidget::editExistingPlot(CustomPlotPtr data)
{
  ui->globalVarsTextField->setPlainText(data->snippet().globalVars);
  ui->mathEquation->setPlainText(data->snippet().function);
  setLinkedPlotName(QString::fromStdString(data->linkedPlotName()));
  ui->pushButtonCreate->setText("Update");
  ui->nameLineEdit->setText(QString::fromStdString(data->name()));
  ui->nameLineEdit->setEnabled(false);

  _is_new = false;

  auto list_widget = ui->listAdditionalSources;
  list_widget->setRowCount(0);

  for (QString curve_name: data->snippet().additionalSources) {
    if( list_widget->findItems(curve_name, Qt::MatchExactly).isEmpty() &&
        curve_name != ui->lineEditSource->text() )
    {
      int row = list_widget->rowCount();
      list_widget->setRowCount(row+1);
      list_widget->setItem(row,0, new QTableWidgetItem( QString("v%1").arg(row+1)));
      list_widget->setItem(row,1, new QTableWidgetItem(curve_name));
    }
  }
  on_listSourcesChanged();

}

//CustomPlotPtr FunctionEditorWidget::getCustomPlotData() const
//{
//  return _plot;
//}

bool FunctionEditorWidget::eventFilter(QObject *obj, QEvent *ev)
{
  if( ev->type() == QEvent::DragEnter )
  {
    auto event = static_cast<QDragEnterEvent*>(ev);
    const QMimeData* mimeData = event->mimeData();
    QStringList mimeFormats = mimeData->formats();

    for(const QString& format : mimeFormats)
    {
      QByteArray encoded = mimeData->data(format);
      QDataStream stream(&encoded, QIODevice::ReadOnly);

      if (format != "curveslist/add_curve")
      {
        return false;
      }

      _dragging_curves.clear();

      while (!stream.atEnd())
      {
        QString curve_name;
        stream >> curve_name;
        if (!curve_name.isEmpty())
        {
          _dragging_curves.push_back(curve_name);
        }
      }
      if( (obj == ui->lineEditSource && _dragging_curves.size() == 1)
      || (obj ==  ui->listAdditionalSources && _dragging_curves.size() > 0) )
      {
        event->acceptProposedAction();
        return true;
      }
    }
  }
  else if ( ev->type() == QEvent::Drop ) {
    if( obj == ui->lineEditSource )
    {
      ui->lineEditSource->setText( _dragging_curves.front() );
    }
    else if ( obj == ui->listAdditionalSources )
    {
      auto list_widget = ui->listAdditionalSources;
      for (QString curve_name: _dragging_curves) {
        if( list_widget->findItems(curve_name, Qt::MatchExactly).isEmpty() &&
            curve_name != ui->lineEditSource->text() )
        {
          int row = list_widget->rowCount();
          list_widget->setRowCount(row+1);
          list_widget->setItem(row,0, new QTableWidgetItem( QString("v%1").arg(row+1)));
          list_widget->setItem(row,1, new QTableWidgetItem(curve_name));
        }
      }   
      on_listSourcesChanged();
    }
  }
  return false;
}

void FunctionEditorWidget::importSnippets(const QByteArray& xml_text)
{
  ui->snippetsListSaved->clear();

  _snipped_saved = GetSnippetsFromXML(xml_text);

  for (const auto& it : _snipped_saved)
  {
    ui->snippetsListSaved->addItem(it.first);
  }

  for (const auto& custom_it : _custom_plots)
  {
    const auto& math_plot = custom_it.second;
    SnippetData snippet;
    snippet.name = QString::fromStdString(math_plot->name());

    if (_snipped_saved.count(snippet.name) > 0)
    {
      continue;
    }

    snippet.globalVars = math_plot->snippet().globalVars;
    snippet.function = math_plot->snippet().function;
  }
  ui->snippetsListSaved->sortItems();
}

QByteArray FunctionEditorWidget::exportSnippets() const
{
  QDomDocument doc;
  auto root = ExportSnippets(_snipped_saved, doc);
  doc.appendChild(root);
  return doc.toByteArray(2);
}

void FunctionEditorWidget::on_snippetsListSaved_currentRowChanged(int current_row)
{
  if (current_row < 0)
  {
    ui->snippetTextEdit->setPlainText("");
    return;
  }
  const auto& name = ui->snippetsListSaved->currentItem()->text();
  const SnippetData& snippet = _snipped_saved.at(name);
  QString desc = QString("%1\n\nfunction calc(time,value)\n\n%2\nend").arg(snippet.globalVars).arg(snippet.function);
  ui->snippetTextEdit->setPlainText(desc);
}

void FunctionEditorWidget::on_snippetsListSaved_doubleClicked(const QModelIndex& index)
{
  const auto& name = ui->snippetsListSaved->item(index.row())->text();
  const SnippetData& snippet = _snipped_saved.at(name);

  ui->globalVarsTextField->setPlainText(snippet.globalVars);
  ui->mathEquation->setPlainText(snippet.function);
}

void FunctionEditorWidget::savedContextMenu(const QPoint& pos)
{
  auto list_saved = ui->snippetsListSaved;

  if (list_saved->selectedItems().size() != 1)
  {
    return;
  }

  QMenu menu;

  QAction* rename_item = new QAction("Rename...", this);
  menu.addAction(rename_item);

  connect(rename_item, &QAction::triggered, this, &FunctionEditorWidget::onRenameSaved);

  QAction* remove_item = new QAction("Remove", this);
  menu.addAction(remove_item);

  connect(remove_item, &QAction::triggered, this, [list_saved, this]() {
    const auto& item = list_saved->selectedItems().first();
    _snipped_saved.erase(item->text());
    delete list_saved->takeItem(list_saved->row(item));
  });

  menu.exec(list_saved->mapToGlobal(pos));
}

void FunctionEditorWidget::on_nameLineEdit_textChanged(const QString& name)
{
  ui->pushButtonCreate->setEnabled(!name.isEmpty());
  ui->pushButtonSave->setEnabled(!name.isEmpty());

  if (_plot_map_data.numeric.count(name.toStdString()) == 0)
  {
    ui->pushButtonCreate->setText("Create New Timeseries");
  }
  else
  {
    ui->pushButtonCreate->setText("Modify Timeseries");
  }
}

void FunctionEditorWidget::on_buttonLoadFunctions_clicked()
{
  QSettings settings;
  QString directory_path = settings.value("AddCustomPlotDialog.loadDirectory", QDir::currentPath()).toString();

  QString fileName =
      QFileDialog::getOpenFileName(this, tr("Open Snippet Library"),
                                   directory_path,
                                   tr("Snippets (*.snippets.xml)"));
  if (fileName.isEmpty())
  {
    return;
  }

  QFile file(fileName);

  if (!file.open(QIODevice::ReadOnly))
  {
    QMessageBox::critical(this, "Error", QString("Failed to open the file [%1]").arg(fileName));
    return;
  }

  directory_path = QFileInfo(fileName).absolutePath();
  settings.setValue("AddCustomPlotDialog.loadDirectory", directory_path);

  importSnippets(file.readAll());
}

void FunctionEditorWidget::on_buttonSaveFunctions_clicked()
{
  QSettings settings;
  QString directory_path = settings.value("AddCustomPlotDialog.loadDirectory",
                                          QDir::currentPath()).toString();

  QString fileName =
      QFileDialog::getSaveFileName(this, tr("Open Snippet Library"),
                                   directory_path,
                                   tr("Snippets (*.snippets.xml)"));

  if (fileName.isEmpty())
  {
    return;
  }
  if (!fileName.endsWith(".snippets.xml"))
  {
    fileName.append(".snippets.xml");
  }

  QFile file(fileName);
  if (!file.open(QIODevice::WriteOnly))
  {
    QMessageBox::critical(this, "Error", QString("Failed to open the file [%1]").arg(fileName));
    return;
  }
  auto data = exportSnippets();

  file.write(data);
  file.close();
}

void FunctionEditorWidget::on_pushButtonSave_clicked()
{
  QString name;

  auto selected_snippets = ui->snippetsListSaved->selectedItems();
  if( selected_snippets.size() >= 1 )
  {
    name = selected_snippets.front()->text();
  }
  bool ok = false;
  name = QInputDialog::getText(this, tr("Name of the Function"),
                               tr("Name:"), QLineEdit::Normal,
                               name, &ok);

  if (!ok || name.isEmpty()) {
    return;
  }

  SnippetData snippet;
  snippet.name = name;
  snippet.globalVars = ui->globalVarsTextField->toPlainText();
  snippet.function = ui->mathEquation->toPlainText();

  addToSaved(name, snippet);

  on_snippetsListSaved_currentRowChanged(ui->snippetsListSaved->currentRow());
}

bool FunctionEditorWidget::addToSaved(const QString& name, const SnippetData& snippet)
{
  if (_snipped_saved.count(name))
  {
    QMessageBox msgBox(this);
    msgBox.setWindowTitle("Warning");
    msgBox.setText(tr("A function with the same name exists already in the list of saved functions.\n"));
    msgBox.addButton(QMessageBox::Cancel);
    QPushButton* button = msgBox.addButton(tr("Overwrite"), QMessageBox::YesRole);
    msgBox.setDefaultButton(button);

    int res = msgBox.exec();

    if (res < 0 || res == QMessageBox::Cancel)
    {
      return false;
    }
  }
  else
  {
    ui->snippetsListSaved->addItem(name);
    ui->snippetsListSaved->sortItems();
  }
  _snipped_saved[name] = snippet;
  return true;
}

void FunctionEditorWidget::onRenameSaved()
{
  auto list_saved = ui->snippetsListSaved;
  auto item = list_saved->selectedItems().first();
  const auto& name = item->text();

  bool ok;
  QString new_name =
      QInputDialog::getText(this, tr("Change the name of the function"), tr("New name:"), QLineEdit::Normal, name, &ok);

  if (!ok || new_name.isEmpty() || new_name == name)
  {
    return;
  }

  SnippetData snippet = _snipped_saved[name];
  _snipped_saved.erase(name);
  snippet.name = new_name;

  _snipped_saved.insert({ new_name, snippet });
  item->setText(new_name);
  ui->snippetsListSaved->sortItems();
}

void FunctionEditorWidget::on_pushButtonCreate_clicked()
{
  try
  {
    std::string plotName = getName().toStdString();

    // check if name is unique (except if is custom_plot)
    if (_plot_map_data.numeric.count(plotName) != 0 && _custom_plots.count(plotName) == 0)
    {
      throw std::runtime_error("plot name already exists and can't be modified");
    }

    if ( _custom_plots.count(plotName) != 0 )
    {
      bool in_additional = ui->listAdditionalSources->findItems(getName(), Qt::MatchExactly).size() > 0;
      if( in_additional )
      {
        throw std::runtime_error("The name of the new plot can not be the same of one "
                                 "of its dependencies in \"Additional time series\"");
      }
      else{
        QMessageBox msgBox(this);
        msgBox.setWindowTitle("Warning");
        msgBox.setText(tr("A custom time series with the same name exists already.\n"
                          " Do you want to overwrite it?\n"));
        msgBox.addButton(QMessageBox::Cancel);
        QPushButton* button = msgBox.addButton(tr("Overwrite"), QMessageBox::YesRole);
        msgBox.setDefaultButton(button);

        int res = msgBox.exec();

        if (res < 0 || res == QMessageBox::Cancel)
        {
          return;
        }
      }
    }

    SnippetData snippet;
    snippet.function = getEquation();
    snippet.globalVars = getGlobalVars();
    snippet.name = getName();
    snippet.linkedSource = getLinkedData();
    for(int row = 0; row < ui->listAdditionalSources->rowCount(); row++)
    {
      snippet.additionalSources.push_back( ui->listAdditionalSources->item(row,1)->text());
    }

    CustomPlotPtr plot = std::make_unique<LuaCustomFunction>(snippet);
    accept(plot);
  }
  catch (const std::runtime_error& e)
  {
    QMessageBox::critical(this, "Error", "Failed to create math plot : " + QString::fromStdString(e.what()));
  }
}


void FunctionEditorWidget::on_pushButtonCancel_pressed()
{
  closed();
}


void FunctionEditorWidget::on_listSourcesChanged()
{
  QString function_text("function( time, value");
  for(int row = 0; row < ui->listAdditionalSources->rowCount(); row++)
  {
    function_text += ", ";
    function_text += ui->listAdditionalSources->item(row,0)->text();
  }
  function_text += " )";
  ui->labelFunction->setText(function_text);
}

void FunctionEditorWidget::on_listAdditionalSources_itemSelectionChanged()
{
    bool any_selected = !ui->listAdditionalSources->selectedItems().isEmpty();
    ui->pushButtonDeleteCurves->setEnabled(any_selected);
}

void FunctionEditorWidget::on_pushButtonDeleteCurves_clicked()
{
  auto list_sources = ui->listAdditionalSources;
  QModelIndexList selected = list_sources->selectionModel()->selectedRows();
  while( selected.size() > 0 )
  {
    list_sources->removeRow( selected.first().row() );
    selected = list_sources->selectionModel()->selectedRows();
  }
  for( int row = 0; row < list_sources->rowCount(); row++ )
  {
    list_sources->item(row,0)->setText( QString("v%1").arg(row+1) );
  }

  on_listAdditionalSources_itemSelectionChanged();
  on_listSourcesChanged();
}

void FunctionEditorWidget::on_lineEditSource_textChanged(const QString &text)
{
  bool valid_source = ( !text.isEmpty() && _plot_map_data.numeric.count(text.toStdString()) > 0);
  ui->pushButtonCreate->setEnabled(valid_source);

}
