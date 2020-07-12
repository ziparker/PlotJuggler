#include "plot_docker.h"
#include <QPushButton>
#include <QBoxLayout>
#include <QMouseEvent>


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
  ads::CDockManager(parent), _name(name)
{
  ads::CDockComponentsFactory::setFactory(new SplittableComponentsFactory());
  ads::CDockManager::setConfigFlag(ads::CDockManager::DockAreaHasTabsMenuButton, false);
  ads::CDockManager::setConfigFlag(ads::CDockManager::DockAreaHasUndockButton, false);
  ads::CDockManager::setConfigFlag(ads::CDockManager::AlwaysShowTabs, true);
  ads::CDockManager::setConfigFlag(ads::CDockManager::EqualSplitOnInsertion, true);

  DockWidget* widget = new DockWidget(datamap, this);

  auto area = addDockWidget(ads::TopDockWidgetArea, widget);
  area->setAllowedAreas(ads::OuterDockAreas);
}

QString PlotDocker::name() const{
  return _name;
}

void PlotDocker::setName(QString name)
{
  _name = name;
}

QDomElement PlotDocker::xmlSaveState(QDomDocument &doc) const
{
  return {};
}

bool PlotDocker::xmlLoadState(QDomElement &element)
{
  return {};
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
  ads::CDockWidget("Plot", parent)
{
  static int plot_count = 0;
  QString plot_name = QString("_plot_%1_").arg(plot_count++);
  auto plot_widget = new PlotWidget(datamap, this);
  setWidget( plot_widget );
  setFeature(ads::CDockWidget::DockWidgetFloatable, false);
  setFeature(ads::CDockWidget::DockWidgetDeleteOnClose, true);

  DraggableToolbar* toolbar = new DraggableToolbar(this);
  toolbar->label()->setText("Plot");
  qobject_cast<QBoxLayout*>(layout())->insertWidget(0, toolbar);

  QObject::connect(toolbar->buttonSplitHorizontal(), &QPushButton::pressed,
                   [&datamap, parent, this]() {
    auto new_widget = new DockWidget(datamap, parent);
    PlotDocker* parent_docker = static_cast<PlotDocker*>( dockManager() );
    auto area = parent_docker->addDockWidget(ads::RightDockWidgetArea,
                                             new_widget, dockAreaWidget());
    area->setAllowedAreas(ads::OuterDockAreas);

    parent_docker->plotWidgetAdded( new_widget->plotWidget() );
  });

  QObject::connect(toolbar->buttonSplitVertical(), &QPushButton::pressed,
                   [&datamap, parent, this]() {
    auto new_widget = new DockWidget(datamap, parent);
    PlotDocker* parent_docker = static_cast<PlotDocker*>( dockManager() );
    auto area = parent_docker->addDockWidget(ads::BottomDockWidgetArea,
                                             new_widget, dockAreaWidget());
    area->setAllowedAreas(ads::OuterDockAreas);

    parent_docker->plotWidgetAdded( new_widget->plotWidget() );
  });

  auto FullscreenAction = [=](bool checked) {
    PlotDocker* parent_docker = static_cast<PlotDocker*>( dockManager() );
    for(int i = 0; i < parent_docker->dockAreaCount(); i++ )
    {
      auto area = parent_docker->dockArea(i);
      if (area != dockAreaWidget())
      {
        area->setVisible(!checked);
      }
    }
  };

  QObject::connect(toolbar->buttonFullscreen(), &QPushButton::toggled, FullscreenAction );
  QObject::connect(toolbar->buttonClose(), &QPushButton::pressed, [=]()
                   { dockAreaWidget()->closeArea();} );


  //this->setMinimumSize( QSize(400,300) );
  this->layout()->setMargin(10);
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
  displayed_toolbar_(false)
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
}

DraggableToolbar::~DraggableToolbar()
{
  delete ui;
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
  ui->widget->setVisible(show);

  QTransform t;
  if( !displayed_toolbar_ )
  {
    t.rotate(-90);
  }

  ui->labelSettings->setPixmap(pixmap.transformed(t));
}

void DraggableToolbar::enterEvent(QEvent *ev)
{
  showToolButtons(true);
  QWidget::enterEvent(ev);
}

void DraggableToolbar::leaveEvent(QEvent *ev)
{
  showToolButtons(false);
  QWidget::leaveEvent(ev);
}


