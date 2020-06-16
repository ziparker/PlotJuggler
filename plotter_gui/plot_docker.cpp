#include "plot_docker.h"
#include "plotwidget.h"
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


PlotDocker::PlotDocker(PlotDataMapRef& datamap, QWidget *parent):
  ads::CDockManager(parent)
{
  ads::CDockComponentsFactory::setFactory(new SplittableComponentsFactory());
  ads::CDockManager::setConfigFlag(ads::CDockManager::DockAreaHasTabsMenuButton, false);
  ads::CDockManager::setConfigFlag(ads::CDockManager::DockAreaHasUndockButton, false);
  ads::CDockManager::setConfigFlag(ads::CDockManager::AlwaysShowTabs, true);

  ads::CDockWidget* widget = new DockWidget(datamap, this);

  auto area = addDockWidget(ads::TopDockWidgetArea, widget);
  area->setAllowedAreas(ads::OuterDockAreas);
}

DockWidget::DockWidget(PlotDataMapRef& datamap, QWidget *parent):
  ads::CDockWidget("Plot", parent)
{
  static int plot_count = 0;
  QString plot_name = QString("_plot_%1_").arg(plot_count++);
  setWidget( new PlotWidget(datamap, this) );
  setFeature(ads::CDockWidget::DockWidgetFloatable, false);
  setFeature(ads::CDockWidget::DockWidgetDeleteOnClose, true);

  DraggableToolbar* toolbar = new DraggableToolbar(this);
  toolbar->label()->setText("Plot");
  qobject_cast<QBoxLayout*>(layout())->insertWidget(0, toolbar);

  QObject::connect(toolbar->buttonSplitHorizontal(), &QPushButton::pressed,
                   [&datamap, parent, this]() {
    auto new_widget = new DockWidget(datamap, parent);
    auto area = dockManager()->addDockWidget(ads::RightDockWidgetArea,
                                             new_widget, dockAreaWidget());
    area->setAllowedAreas(ads::OuterDockAreas);
  });

  QObject::connect(toolbar->buttonSplitVertical(), &QPushButton::pressed,
                   [&datamap, parent, this]() {
    auto new_widget = new DockWidget(datamap, parent);
    auto area = dockManager()->addDockWidget(ads::BottomDockWidgetArea,
                                             new_widget, dockAreaWidget());
    area->setAllowedAreas(ads::OuterDockAreas);
  });

  auto FullscreenAction = [=](bool checked) {
    auto manager = dockManager();
    for(int i = 0; i < manager->dockAreaCount(); i++ )
    {
      auto area = manager->dockArea(i);
      if (area != dockAreaWidget())
      {
        area->setVisible(!checked);
      }
    }
  };

  QObject::connect(toolbar->buttonFullscreen(), &QPushButton::toggled, FullscreenAction );
  QObject::connect(toolbar->buttonClose(), &QPushButton::pressed, [=]()
                   { dockAreaWidget()->closeArea();} );
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
  setButtonIcon(ui->buttonFunction, ":/resources/svg/function.svg");
  setButtonIcon(ui->buttonFullscreen, ":/resources/svg/fullscreen.svg");
  setButtonIcon(ui->buttonZoomOut,  ":/resources/svg/zoom_max.svg");
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


