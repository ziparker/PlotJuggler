#include "plot_docker.h"
#include "plotwidget.h"
#include <QPushButton>
#include <QBoxLayout>

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
}

DockWidget::DockWidget(PlotDataMapRef& datamap, QWidget *parent):
  ads::CDockWidget("Plot", parent)
{
/*  static int plot_count = 0;
  QString plot_name = QString("_plot_%1_").arg(plot_count++);
  setWidget( new PlotWidget(datamap, this) );
  setFeature(ads::CDockWidget::DockWidgetFloatable, false);
  setFeature(ads::CDockWidget::DockWidgetDeleteOnClose, true);

  DraggableToolbar* toolbar = new DraggableToolbar(widget);
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
                   { dockAreaWidget()->closeArea();} );*/
}
