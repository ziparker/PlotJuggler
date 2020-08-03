#include <QMenu>
#include <QSignalMapper>
#include <QAction>
#include <QTabBar>
#include <QSvgGenerator>
#include <QInputDialog>
#include <QMouseEvent>
#include <QFileDialog>
#include <QApplication>
#include <QPainter>
#include <QTabWidget>
#include <QPushButton>
#include <QHBoxLayout>
#include "qwt_plot_renderer.h"
#include "mainwindow.h"
#include "tabbedplotwidget.h"
#include "tab_widget.h"
#include "svg_util.h"

std::map<QString, TabbedPlotWidget*> TabbedPlotWidget::_instances;

TabbedPlotWidget::TabbedPlotWidget(QString name, QMainWindow* mainwindow,
                                   PlotDataMapRef& mapped_data, QMainWindow* parent)
  : QWidget(parent)
  , _mapped_data(mapped_data)
  , _name(name)
  , _main_window(mainwindow)
  , _labels_status(LabelStatus::RIGHT)
{
  MainWindow* main_window = dynamic_cast<MainWindow*>(_main_window);

  setContentsMargins(4,0,6,6);

  if (main_window == parent)
  {
    _parent_type = "main_window";
  }
  else
  {
    _parent_type = "floating_window";
  }

  if (TabbedPlotWidget::_instances.count(_name) > 0)
  {
    throw std::runtime_error("This is not supposed to happen");
  }
  // register this instance
  _instances[_name] = this;

  _horizontal_link = true;

  QHBoxLayout* main_layout = new QHBoxLayout(this);
  main_layout->setMargin(0);

  _tabWidget = new QTabWidget(this);
  _tabWidget->setTabsClosable(true);
  _tabWidget->setMovable(true);

  connect(_tabWidget->tabBar(), &QTabBar::tabBarDoubleClicked,
          this, &TabbedPlotWidget::on_renameCurrentTab);

  main_layout->addWidget(_tabWidget);

  connect(_tabWidget, &QTabWidget::currentChanged,
          this, &TabbedPlotWidget::on_tabWidget_currentChanged);

  tabWidget()->tabBar()->installEventFilter(this);


  // TODO _action_savePlots = new QAction(tr("&Save plots to file"), this);
  // TODO connect(_action_savePlots, &QAction::triggered, this, &TabbedPlotWidget::on_savePlotsToFile);

//  _tab_menu = new QMenu(this);
//  _tab_menu->addSeparator();
//  //_tab_menu->addAction(_action_savePlots);
//  _tab_menu->addSeparator();

  connect(this, &TabbedPlotWidget::destroyed, main_window, &MainWindow::on_tabbedAreaDestroyed);
  connect(this, &TabbedPlotWidget::tabAdded, main_window, &MainWindow::onPlottabAdded);
  connect(this, &TabbedPlotWidget::undoableChange, main_window, &MainWindow::onUndoableChange);

  // TODO connect(_tabWidget, &TabWidget::movingPlotWidgetToTab, this, &TabbedPlotWidget::onMoveWidgetIntoNewTab);

  this->addTab({});

  _buttonAddTab = new QPushButton("",this);
  _buttonAddTab->setFlat(true);
  _buttonAddTab->setFixedSize( QSize(32,32));
  _buttonAddTab->setFocusPolicy(Qt::NoFocus);

  connect(_buttonAddTab, &QPushButton::pressed, this, &TabbedPlotWidget::on_addTabButton_pressed);
}

void TabbedPlotWidget::paintEvent(QPaintEvent *event)
{
  QWidget::paintEvent(event);

  auto size = tabWidget()->tabBar()->size();
  _buttonAddTab->move( QPoint( size.width()+ 5, 0) );
}


PlotDocker* TabbedPlotWidget::currentTab()
{
  return static_cast<PlotDocker*>(tabWidget()->currentWidget());
}

QTabWidget* TabbedPlotWidget::tabWidget()
{
  return _tabWidget;
}

const QTabWidget* TabbedPlotWidget::tabWidget() const
{
  return _tabWidget;
}

PlotDocker* TabbedPlotWidget::addTab(QString tab_name)
{
  static int tab_suffix_count = 1;

  // this must be done before ant PlotDocker is created
  ads::CDockManager::setConfigFlag(ads::CDockManager::DockAreaHasTabsMenuButton, false);
  ads::CDockManager::setConfigFlag(ads::CDockManager::DockAreaHasUndockButton, false);
  ads::CDockManager::setConfigFlag(ads::CDockManager::AlwaysShowTabs, true);
  ads::CDockManager::setConfigFlag(ads::CDockManager::EqualSplitOnInsertion, true);
  ads::CDockManager::setConfigFlag(ads::CDockManager::OpaqueSplitterResize, true);

  if( tab_name.isEmpty() )
  {
    tab_name = QString("tab%1").arg(tab_suffix_count++);
  }

  auto docker = new PlotDocker(tab_name, _mapped_data, this);
  connect( docker, &PlotDocker::undoableChange, this, &TabbedPlotWidget::undoableChange );

  tabWidget()->addTab(docker, tab_name);

  emit tabAdded(docker);
  // we need to send the signal for the very first widget
  emit docker->plotWidgetAdded( docker->plotAt(0) );

  int index = tabWidget()->count() - 1;

  QWidget* button_widget = new QWidget();
  QHBoxLayout* layout = new QHBoxLayout(button_widget);
  layout->setSpacing(2);
  layout->setMargin(0);

  QPushButton* close_button = new QPushButton();
  close_button->setIcon(LoadSvgIcon(":/resources/svg/close-button.svg", "light"));
  close_button->setFixedSize( QSize(16,16));
  close_button->setFlat(true);
  connect(close_button, &QPushButton::pressed,
          this, [this](){ on_tabWidget_tabCloseRequested(tabWidget()->tabBar()->currentIndex());} );

  layout->addWidget(close_button);
  tabWidget()->tabBar()->setTabButton(index, QTabBar::RightSide, button_widget);

  docker->setHorizontalLink(_horizontal_link);

  tabWidget()->setCurrentWidget(docker);

  return docker;
}

QDomElement TabbedPlotWidget::xmlSaveState(QDomDocument& doc) const
{
  QDomElement tabbed_area = doc.createElement("tabbed_widget");

  tabbed_area.setAttribute("name", _name);
  tabbed_area.setAttribute("parent", _parent_type);

  for (int i = 0; i < tabWidget()->count(); i++)
  {
    PlotDocker* widget = static_cast<PlotDocker*>(tabWidget()->widget(i));
    QDomElement element = widget->xmlSaveState(doc);

    element.setAttribute("tab_name", tabWidget()->tabText(i));
    tabbed_area.appendChild(element);
  }

  QDomElement current_plotmatrix = doc.createElement("currentTabIndex");
  current_plotmatrix.setAttribute("index", tabWidget()->currentIndex());
  tabbed_area.appendChild(current_plotmatrix);

  return tabbed_area;
}

bool TabbedPlotWidget::xmlLoadState(QDomElement& tabbed_area)
{
  int prev_count = tabWidget()->count();

  for (auto docker_elem = tabbed_area.firstChildElement("Tab");
       !docker_elem.isNull();
       docker_elem = docker_elem.nextSiblingElement("Tab"))
  {
    QString tab_name = docker_elem.attribute("tab_name");
    PlotDocker* docker = addTab( tab_name );

    bool success = docker->xmlLoadState(docker_elem);

    if (!success)
    {
      return false;
    }
  }

  // remove old ones
  for(int i=0; i<prev_count; i++ )
  {
    tabWidget()->removeTab(0);
  }

  QDomElement current_tab = tabbed_area.firstChildElement("currentTabIndex");
  int current_index = current_tab.attribute("index").toInt();

  if (current_index >= 0 && current_index < tabWidget()->count())
  {
    tabWidget()->setCurrentIndex(current_index);
  }

  emit undoableChange();
  return true;
}

void TabbedPlotWidget::setStreamingMode(bool streaming_mode)
{

}

TabbedPlotWidget::~TabbedPlotWidget()
{

}

void TabbedPlotWidget::on_renameCurrentTab()
{
  int idx = tabWidget()->tabBar()->currentIndex();

  bool ok = true;
  QString newName = QInputDialog::getText(this, tr("Change the tab name"), tr("New name:"),
                                          QLineEdit::Normal, tabWidget()->tabText(idx), &ok);
  if (ok)
  {
    tabWidget()->setTabText(idx, newName);
    currentTab()->setName(newName);
  }
}

/*void TabbedPlotWidget::on_savePlotsToFile()
{
  int idx = tabWidget()->tabBar()->currentIndex();
  PlotDocker* matrix = static_cast<PlotDocker*>(tabWidget()->widget(idx));

  QFileDialog saveDialog(this);
  saveDialog.setAcceptMode(QFileDialog::AcceptSave);
  saveDialog.selectFile(currentTab()->name());

  QStringList filters;
  filters << "png (*.png)"
          << "jpg (*.jpg *.jpeg)"
          << "svg (*.svg)";

  saveDialog.setNameFilters(filters);
  saveDialog.exec();

  if (saveDialog.result() == QDialog::Accepted && !saveDialog.selectedFiles().empty())
  {
    QString fileName = saveDialog.selectedFiles().first();

    QFileInfo fileinfo(fileName);
    if (fileinfo.suffix().isEmpty())
    {
      auto filter = saveDialog.selectedNameFilter();
      if (filter == filters[0])
      {
        fileName.append(".png");
      }
      else if (filter == filters[1])
      {
        fileName.append(".jpg");
      }
      else if (filter == filters[2])
      {
        fileName.append(".svg");
      }
    }
    //saveTabImage(fileName, matrix);
  }
}*/

/*void TabbedPlotWidget::saveTabImage(QString fileName, PlotDocker *matrix)
{
  bool is_svg = (QFileInfo(fileName).suffix().toLower() == "svg");

  QPixmap pixmap(1200, 900);
  QRect documentRect(0, 0, 1200, 900);

  QSvgGenerator generator;
  QPainter* painter = nullptr;

  if (is_svg)
  {
    generator.setFileName(fileName);
    generator.setResolution(80);
    generator.setViewBox(documentRect);
    painter = new QPainter(&generator);
  }
  else
  {
    painter = new QPainter(&pixmap);
  }

  if (fileName.isEmpty())
  {
    return;
  }

  QwtPlotRenderer rend;

  int delta_X = pixmap.width() / matrix->colsCount();
  int delta_Y = pixmap.height() / matrix->rowsCount();

  for (unsigned c = 0; c < matrix->colsCount(); c++)
  {
    for (unsigned r = 0; r < matrix->rowsCount(); r++)
    {
      PlotWidget* widget = matrix->plotAt(r, c);
      bool tracker_enabled = widget->isTrackerEnabled();
      if (tracker_enabled)
      {
        widget->enableTracker(false);
        widget->replot();
      }

      QRect rect(delta_X * c, delta_Y * r, delta_X, delta_Y);
      rend.render(widget, painter, rect);

      if (tracker_enabled)
      {
        widget->enableTracker(true);
        widget->replot();
      }
    }
  }
  painter->end();
  if (!is_svg)
  {
    pixmap.save(fileName);
  }
}
*/

void TabbedPlotWidget::on_stylesheetChanged(QString style_dir)
{
  // TODO
  _buttonAddTab->setIcon(LoadSvgIcon(":/resources/svg/add_tab.svg", style_dir));
  //ui->pushButtonShowLabel->setIcon(LoadSvgIcon(":/resources/svg/legend.svg", style_dir));
  //_button_link_horizontal->setIcon(LoadSvgIcon(":/resources/svg/link.svg", style_dir));
}

/*

void TabbedPlotWidget::onMoveWidgetIntoNewTab(QString plot_name)
{
  int src_row, src_col;
  PlotDocker* src_matrix = nullptr;
  PlotWidget* source = nullptr;

  for (int t = 0; t < tabWidget()->count(); t++)
  {
    PlotDocker* matrix = static_cast<PlotDocker*>(tabWidget()->widget(t));

    for (unsigned row = 0; row < matrix->rowsCount(); row++)
    {
      for (unsigned col = 0; col < matrix->colsCount(); col++)
      {
        PlotWidget* plot = matrix->plotAt(row, col);
        if (plot->windowTitle() == plot_name)
        {
          src_matrix = matrix;
          src_row = row;
          src_col = col;
          source = plot;
          break;
        }
      }
    }
  }

  addTab();
  PlotDocker* dst_matrix = currentTab();
  PlotWidget* destination = dst_matrix->plotAt(0, 0);

  src_matrix->gridLayout()->removeWidget(source);
  dst_matrix->gridLayout()->removeWidget(destination);

  src_matrix->gridLayout()->addWidget(destination, src_row, src_col);
  dst_matrix->gridLayout()->addWidget(source, 0, 0);
  source->changeBackgroundColor(Qt::white);
  destination->changeBackgroundColor(Qt::white);

  src_matrix->removeEmpty();
  src_matrix->updateLayout();
  dst_matrix->updateLayout();
  emit undoableChange();
}*/

void TabbedPlotWidget::on_addTabButton_pressed()
{
  addTab(nullptr);
  emit undoableChange();
}

void TabbedPlotWidget::on_tabWidget_currentChanged(int index)
{
  if (tabWidget()->count() == 0)
  {
    if (_parent_type.compare("main_window") == 0)
    {
      addTab(nullptr);
    }
    else
    {
      this->parent()->deleteLater();
    }
  }

  PlotDocker* tab = dynamic_cast<PlotDocker*>(tabWidget()->widget(index));
  if (tab)
  {
    tab->replot();
  }
  for (int i=0; i<tabWidget()->count(); i++ )
  {
    auto button = _tabWidget->tabBar()->tabButton(i, QTabBar::RightSide);
    if( button ){
      button->setHidden( i!=index );
    }
  }
}

void TabbedPlotWidget::on_tabWidget_tabCloseRequested(int index)
{
  PlotDocker* tab = dynamic_cast<PlotDocker*>( tabWidget()->widget(index) );

  // first add then delete.
  // Otherwise currentPlotGrid might be empty
  if (tabWidget()->count() == 1)
  {
    on_addTabButton_pressed();
  }

  PlotDocker* docker = static_cast<PlotDocker*>(tabWidget()->widget(index));

  for (unsigned p = 0; p < docker->plotCount(); p++)
  {
    PlotWidget* plot = docker->plotAt(p);
    plot->detachAllCurves();
    plot->deleteLater();
  }
  docker->deleteLater();

  tabWidget()->removeTab(index);
  emit undoableChange();

}

void TabbedPlotWidget::on_buttonLinkHorizontalScale_toggled(bool checked)
{
  _horizontal_link = checked;

  for (int i = 0; i < tabWidget()->count(); i++)
  {
    PlotDocker* tab = static_cast<PlotDocker*>(tabWidget()->widget(i));
    tab->setHorizontalLink(_horizontal_link);
  }
}

void TabbedPlotWidget::on_requestTabMovement(const QString& destination_name)
{
  TabbedPlotWidget* destination_widget = TabbedPlotWidget::_instances[destination_name];

  PlotDocker* tab_to_move = currentTab();
  int index = tabWidget()->tabBar()->currentIndex();

  const QString& tab_name = this->tabWidget()->tabText(index);

  destination_widget->tabWidget()->addTab(tab_to_move, tab_name);
  emit undoableChange();
}

void TabbedPlotWidget::on_moveTabIntoNewWindow()
{
  emit sendTabToNewWindow(currentTab());
}

void TabbedPlotWidget::on_pushButtonShowLabel_pressed()
{
  switch (_labels_status)
  {
    case LabelStatus::LEFT:
      _labels_status = LabelStatus::HIDDEN;
      break;
    case LabelStatus::RIGHT:
      _labels_status = LabelStatus::LEFT;
      break;
    case LabelStatus::HIDDEN:
      _labels_status = LabelStatus::RIGHT;
      break;
  }
  onLabelStatusChanged();
}

bool TabbedPlotWidget::eventFilter(QObject* obj, QEvent* event)
{
  QTabBar* tab_bar = tabWidget()->tabBar();

  if (obj == tab_bar)
  {
    if (event->type() == QEvent::MouseButtonPress)
    {
      QMouseEvent* mouse_event = static_cast<QMouseEvent*>(event);

      int index = tab_bar->tabAt(mouse_event->pos());
      tab_bar->setCurrentIndex(index);

      if (mouse_event->button() == Qt::RightButton)
      {
        //QMenu* submenu = new QMenu("Move tab to...");
       // _tab_menu->addMenu(submenu);

       // QSignalMapper* signalMapper = new QSignalMapper(submenu);

        //-----------------------------------
//        QAction* action_new_window = submenu->addAction("New Window");
//        submenu->addSeparator();
//        connect(action_new_window, &QAction::triggered, this, &TabbedPlotWidget::on_moveTabIntoNewWindow);

//        //-----------------------------------
//        for (auto& it : TabbedPlotWidget::_instances)
//        {
//          QString name = it.first;
//          TabbedPlotWidget* tabbed_menu = it.second;
//          if (tabbed_menu != this)
//          {
//            QAction* action = submenu->addAction(name);
//            connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
//            signalMapper->setMapping(action, name);
//          }
//        }

//        connect(signalMapper, SIGNAL(mapped(QString)), this, SLOT(on_requestTabMovement(QString)));

//        //-------------------------------
////        QIcon iconSave;
////        iconSave.addFile(tr(":/%1/save.png").arg(theme), QSize(26, 26));
////        _action_savePlots->setIcon(iconSave);

////        QIcon iconNewWin;
////        iconNewWin.addFile(tr(":/%1/stacks.png").arg(theme), QSize(16, 16));
////        action_new_window->setIcon(iconNewWin);

//        _tab_menu->exec(mouse_event->globalPos());
//        //-------------------------------
//        submenu->deleteLater();
      }
    }
  }

  // Standard event processing
  return QObject::eventFilter(obj, event);
}

void TabbedPlotWidget::onLabelStatusChanged()
{
  for (int i = 0; i < tabWidget()->count(); i++)
  {
    PlotDocker* matrix = static_cast<PlotDocker*>(tabWidget()->widget(i));

    for (unsigned p = 0; p < matrix->plotCount(); p++)
    {
      PlotWidget* plot = matrix->plotAt(p);

      plot->activateLegend(_labels_status != LabelStatus::HIDDEN);
      if (_labels_status == LabelStatus::LEFT)
      {
        plot->setLegendAlignment(Qt::AlignLeft);
      }
      else if (_labels_status == LabelStatus::RIGHT)
      {
        plot->setLegendAlignment(Qt::AlignRight);
      }
      plot->replot();
    }
  }
}


void TabbedPlotWidget::closeEvent(QCloseEvent* event)
{
  TabbedPlotWidget::_instances.erase(name());
}

const std::map<QString, TabbedPlotWidget*>& TabbedPlotWidget::instances()
{
  return TabbedPlotWidget::_instances;
}

TabbedPlotWidget* TabbedPlotWidget::instance(const QString& key)
{
  auto it = TabbedPlotWidget::_instances.find(key);
  if (it == TabbedPlotWidget::_instances.end())
  {
    return nullptr;
  }
  else
  {
    return it->second;
  }
}

void TabbedPlotWidget::setControlsVisible(bool visible)
{
  // ui->widgetControls->setVisible(visible);
}
