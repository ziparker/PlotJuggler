#include "plot_docker.h"
#include "plotwidget_editor.h"
#include "Qads/DockSplitter.h"
#include <QPushButton>
#include <QBoxLayout>
#include <QMouseEvent>
#include <QSplitter>
#include <QDebug>
#include <QInputDialog>
#include <QLineEdit>


class SplittableComponentsFactory : public ads::CDockComponentsFactory
{
public:
  ads::CDockAreaTitleBar* createDockAreaTitleBar(ads::CDockAreaWidget* dock_area) const override
  {
    auto title_bar =  new ads::CDockAreaTitleBar(dock_area);
    title_bar->setVisible(false);
    return title_bar;
  }
};


PlotDocker::PlotDocker(QString name, PlotDataMapRef& datamap, QWidget *parent):
  ads::CDockManager(parent), _name(name), _datamap(datamap)
{
  ads::CDockComponentsFactory::setFactory(new SplittableComponentsFactory());

  auto CreateFirstWidget = [&]()
  {
    if( dockAreaCount() == 0)
    {
      DockWidget* widget = new DockWidget(datamap, this);

      auto area = addDockWidget(ads::TopDockWidgetArea, widget);
      area->setAllowedAreas(ads::OuterDockAreas);

      connect(widget, &DockWidget::undoableChange, this, &PlotDocker::undoableChange);
    }
  };

  connect(this, &ads::CDockManager::dockWidgetRemoved, this, CreateFirstWidget);

  connect(this, &ads::CDockManager::dockAreasAdded, this, &PlotDocker::undoableChange);

  CreateFirstWidget();
}

QString PlotDocker::name() const{
  return _name;
}

void PlotDocker::setName(QString name)
{
  _name = name;
}


QDomElement saveChildNodesState(QDomDocument &doc, QWidget* widget)
{
    QSplitter* splitter = qobject_cast<QSplitter*>(widget);
    if (splitter)
    {
      QDomElement splitter_elem = doc.createElement("DockSplitter");
      splitter_elem.setAttribute("orientation", (splitter->orientation() == Qt::Horizontal) ? "|" : "-");
      splitter_elem.setAttribute("count", QString::number(splitter->count()));

      QString sizes_str;
      int total_size = 0;
      for(int size : splitter->sizes() )
      {
        total_size += size;
      }
      for(int size : splitter->sizes() )
      {
        sizes_str += QString::number( double(size)/double(total_size) );
        sizes_str += ";";
      }
      sizes_str.resize( sizes_str.size()-1 );
      splitter_elem.setAttribute("sizes", sizes_str);

      for (int i = 0; i < splitter->count(); ++i)
      {
        auto child = saveChildNodesState(doc, splitter->widget(i));
        splitter_elem.appendChild( child );
      }
      return splitter_elem;
    }
    else{
      ads::CDockAreaWidget* dockArea = qobject_cast<ads::CDockAreaWidget*>(widget);
      if (dockArea)
      {
        QDomElement area_elem = doc.createElement("DockArea");
        for (int i = 0; i < dockArea->dockWidgetsCount(); ++i)
        {
          auto dock_widget = dynamic_cast<DockWidget*>( dockArea->dockWidget(i) );
          if( dock_widget )
          {
            auto plotwidget_elem = dock_widget->plotWidget()->xmlSaveState(doc);
            area_elem.appendChild( plotwidget_elem );
            area_elem.setAttribute("name", dock_widget->toolBar()->label()->text());
          }
        }
        return area_elem;
      }
    }
    return {};
}

QDomElement PlotDocker::xmlSaveState(QDomDocument &doc) const
{
  QDomElement containers_elem = doc.createElement("Tab");

  containers_elem.setAttribute("containers", dockContainers().count());

  for (CDockContainerWidget* container : dockContainers())
  {
    QDomElement elem = doc.createElement("Container");
    auto child = saveChildNodesState(doc, container->rootSplitter());
    elem.appendChild(child);
    containers_elem.appendChild(elem);
  }
  return containers_elem;
}

void PlotDocker::restoreSplitter(QDomElement elem, DockWidget* widget)
{
  QString orientation_str = elem.attribute("orientation");
  int splitter_count = elem.attribute("count").toInt();

  // Check if the orientation string is right
  if (!orientation_str.startsWith("|") && !orientation_str.startsWith("-"))
  {
    return;
  }

  Qt::Orientation orientation = orientation_str.startsWith("|") ? Qt::Horizontal : Qt::Vertical;

  std::vector<DockWidget*> widgets(splitter_count);

  widgets[0] = widget;
  for (int i=1; i<splitter_count; i++ )
  {
    widget = (orientation == Qt::Horizontal) ?
                 widget->spliHorizontal() : widget->spliVertical();
    widgets[i] = widget;
  }

  int tot_size = 0;

  for (int i=0; i<splitter_count; i++ )
  {
    tot_size += ( orientation == Qt::Horizontal ) ? widgets[i]->width() : widgets[i]->height();
  }

  auto sizes_str = elem.attribute("sizes").splitRef(";", QString::SkipEmptyParts);
  QList<int> sizes;

  for (int i=0; i<splitter_count; i++ )
  {
    sizes.push_back( static_cast<int>(sizes_str[i].toDouble() * tot_size) );
  }

  auto splitter = ads::internal::findParent<ads::CDockSplitter*>(widget);
  splitter->setSizes(sizes);

  int index = 0;

  QDomElement child_elem = elem.firstChildElement();
  while (child_elem.isNull() == false)
  {
    if( child_elem.tagName() == "DockArea" )
    {
      auto plot_elem = child_elem.firstChildElement("plot");
      widgets[index]->plotWidget()->xmlLoadState(plot_elem);
      if( child_elem.hasAttribute("name") )
      {
        QString area_name = child_elem.attribute("name");
        widgets[index]->toolBar()->label()->setText(area_name);
      }
      index++;
    }
    else if( child_elem.tagName() == "DockSplitter" )
    {
       restoreSplitter(child_elem, widgets[index++]);
    }
    else{
      return;
    }

    child_elem = child_elem.nextSiblingElement();
  }
};

bool PlotDocker::xmlLoadState(QDomElement &tab_element)
{
  if (!isHidden())
  {
    hide();
  }

  for (auto container_elem = tab_element.firstChildElement("Container");
       !container_elem.isNull();
       container_elem = container_elem.nextSiblingElement("Container"))
  {
    auto splitter_elem = container_elem.firstChildElement("DockSplitter");
    if( !splitter_elem.isNull())
    {
      auto widget = dynamic_cast<DockWidget*>( dockArea(0)->currentDockWidget());
      restoreSplitter(splitter_elem, widget );
    }
  }

  if (isHidden())
  {
    show();
  }
  return true;
}

int PlotDocker::plotCount() const
{
  return  dockAreaCount();
}

PlotWidget *PlotDocker::plotAt(int index)
{
  return static_cast<PlotWidget*>( dockArea(index)->currentDockWidget()->widget() );
}

void PlotDocker::setHorizontalLink(bool enabled)
{
  // TODO
}

void PlotDocker::zoomOut()
{
  for (int index = 0; index < plotCount(); index++)
  {
    plotAt(index)->zoomOut(false); // TODO is it false?
  }
}

void PlotDocker::replot()
{
  for (int index = 0; index < plotCount(); index++)
  {
    plotAt(index)->replot();
  }
}

DockWidget::DockWidget(PlotDataMapRef& datamap, QWidget *parent):
  ads::CDockWidget("Plot", parent), _datamap(datamap)
{
  setFrameShape(QFrame::NoFrame);

  static int plot_count = 0;
  QString plot_name = QString("_plot_%1_").arg(plot_count++);
  auto plot_widget = new PlotWidget(datamap, this);
  setWidget( plot_widget );
  setFeature(ads::CDockWidget::DockWidgetFloatable, false);
  setFeature(ads::CDockWidget::DockWidgetDeleteOnClose, true);

  _toolbar = new DraggableToolbar(this);
  _toolbar->label()->setText("*");
  qobject_cast<QBoxLayout*>(layout())->insertWidget(0, _toolbar);

  QObject::connect(_toolbar->buttonSplitHorizontal(), &QPushButton::pressed,
                   this, &DockWidget::spliHorizontal);

  QObject::connect(_toolbar->buttonSplitVertical(), &QPushButton::pressed,
                   this, &DockWidget::spliVertical);

  auto FullscreenAction = [=](bool is_fullscreen) {
    PlotDocker* parent_docker = static_cast<PlotDocker*>( dockManager() );
    for(int i = 0; i < parent_docker->dockAreaCount(); i++ )
    {
      auto area = parent_docker->dockArea(i);
      if (area != dockAreaWidget())
      {
        area->setVisible(!is_fullscreen);
      }
      this->toolBar()->buttonClose()->setHidden(is_fullscreen);
      this->toolBar()->toggleFullscreen(is_fullscreen);
    }
  };

  QObject::connect(_toolbar->buttonFullscreen(), &QPushButton::toggled, FullscreenAction );

  QObject::connect(_toolbar->buttonClose(), &QPushButton::pressed, [=]()
                   {
                     dockAreaWidget()->closeArea();
                     this->undoableChange();
                   } );

  _toolbar->buttonEdit()->setCheckable(false);
  QObject::connect(_toolbar->buttonEdit(), &QPushButton::clicked, this, [=]()
                   {
                     auto editor_dialog = new PlotwidgetEditor(plot_widget, this);
                     editor_dialog->exec();
                     editor_dialog->deleteLater();
                     _toolbar->showToolButtons(false);
                   } );

  //this->setMinimumSize( QSize(400,300) );
  this->layout()->setMargin(10);
}

DockWidget* DockWidget::spliHorizontal()
{
  // create a sibling (same parent)
  auto new_widget = new DockWidget(_datamap, qobject_cast<QWidget*>(parent()));

  PlotDocker* parent_docker = static_cast<PlotDocker*>( dockManager() );
  auto area = parent_docker->addDockWidget(ads::RightDockWidgetArea,
                                           new_widget, dockAreaWidget());

  area->setAllowedAreas(ads::OuterDockAreas);

  parent_docker->plotWidgetAdded( new_widget->plotWidget() );

  connect(this, &DockWidget::undoableChange, parent_docker, &PlotDocker::undoableChange);

  return new_widget;
}

DockWidget* DockWidget::spliVertical()
{
  auto new_widget = new DockWidget(_datamap, qobject_cast<QWidget*>(parent()));

  PlotDocker* parent_docker = static_cast<PlotDocker*>( dockManager() );

  auto area = parent_docker->addDockWidget(ads::BottomDockWidgetArea,
                                           new_widget, dockAreaWidget());

  area->setAllowedAreas(ads::OuterDockAreas);
  parent_docker->plotWidgetAdded( new_widget->plotWidget() );

  connect(this, &DockWidget::undoableChange, parent_docker, &PlotDocker::undoableChange);

  return new_widget;
}

PlotWidget *DockWidget::plotWidget()
{
  return static_cast<PlotWidget*>( widget() );
}

DraggableToolbar *DockWidget::toolBar()
{
  return _toolbar;
}

static void setButtonIcon(QPushButton* button, const QString& file)
{
  QIcon icon(file);
  icon.addPixmap(icon.pixmap(92));
  button->setIcon(icon);
  button->setText("");
}

DraggableToolbar::DraggableToolbar(ads::CDockWidget* parent) :
  QWidget(parent),
  parent_(parent),
  ui(new Ui::DraggableToolbar),
  displayed_toolbar_(false),
  fullscreen_mode_(false)
{
  ui->setupUi(this);

  setButtonIcon(ui->buttonSplitHorizontal, ":/resources/svg/add_column.svg");
  setButtonIcon(ui->buttonSplitVertical, ":/resources/svg/add_row.svg");
  setButtonIcon(ui->buttonFullscreen, ":/resources/svg/fullscreen.svg");
  setButtonIcon(ui->buttonEdit, ":/resources/svg/pencil-edit.svg");
  setButtonIcon(ui->buttonClose,  ":/resources/svg/close-button.svg");

  setMouseTracking(true);
  ui->widgetButtons->setMouseTracking(true);
  ui->buttonClose->setMouseTracking(true);
  ui->labelSettings->setMouseTracking(true);

  displayed_toolbar_ = true;
  showToolButtons(false);

  ui->label->installEventFilter(this);
}

DraggableToolbar::~DraggableToolbar()
{
  delete ui;
}

void DraggableToolbar::toggleFullscreen(bool is_fullscreen)
{
  fullscreen_mode_ = is_fullscreen;
  ui->labelSettings->setHidden(is_fullscreen);
  ui->buttonClose->setHidden(is_fullscreen);
  ui->widgetButtons->setHidden(is_fullscreen);

  QHBoxLayout* layout = ui->mainHorizontalLayout;
  QHBoxLayout* button_layout = ui->buttonHorizontalLayout;

  if( is_fullscreen ){
    button_layout->removeWidget( ui->buttonFullscreen );
    layout->addWidget( ui->buttonFullscreen );
  }
  else{
    layout->removeWidget( ui->buttonFullscreen );
    button_layout->addWidget( ui->buttonFullscreen );
  }
}

void DraggableToolbar::mousePressEvent(QMouseEvent *ev)
{
  parent_->dockAreaWidget()->titleBar()->mousePressEvent(ev);
}

void DraggableToolbar::mouseReleaseEvent(QMouseEvent *ev)
{
  parent_->dockAreaWidget()->titleBar()->mouseReleaseEvent(ev);
}

void DraggableToolbar::mouseMoveEvent(QMouseEvent *ev)
{
  auto pos = buttonClose()->mapFromParent(ev->pos());
  bool under_mouse = buttonClose()->rect().contains( pos );
  //qDebug() << pos << ": " << under_mouse << " / " << displayed_toolbar_;
  showToolButtons( !under_mouse );

  parent_->dockAreaWidget()->titleBar()->mouseMoveEvent(ev);

  ev->accept();
  QWidget::mouseMoveEvent(ev);
}

void DraggableToolbar::showToolButtons(bool show)
{
  static auto icon = QIcon(":/resources/svg/left-arrow.svg");
  auto pixmap = icon.pixmap(14);

  if( show == displayed_toolbar_)
  {
    return;
  }
  displayed_toolbar_ = show;
  ui->buttonEdit->setVisible(show);
  ui->buttonSplitVertical->setVisible(show);
  ui->buttonSplitHorizontal->setVisible(show);
  ui->buttonFullscreen->setVisible(show || fullscreen_mode_ );

  QTransform t;
  if( !displayed_toolbar_ )
  {
    t.rotate(-90);
  }

  ui->labelSettings->setPixmap(pixmap.transformed(t));
}

bool DraggableToolbar::eventFilter(QObject *object, QEvent *event)
{
  if(event->type() == QEvent::MouseButtonDblClick)
  {
    bool ok = true;
    QString newName = QInputDialog::getText(this, tr("Change name of the Area"), tr("New name:"),
                                            QLineEdit::Normal, ui->label->text(), &ok);
    if (ok)
    {
      ui->label->setText(newName);
    }
    return true;
  }
  else{
    return QObject::eventFilter(object,event);
  }
}

void DraggableToolbar::leaveEvent(QEvent *ev)
{
  showToolButtons(false);
  QWidget::leaveEvent(ev);
}


