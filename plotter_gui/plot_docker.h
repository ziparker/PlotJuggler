#ifndef PLOT_DOCKER_H
#define PLOT_DOCKER_H

#include "Qads/DockManager.h"
#include "Qads/DockWidget.h"
#include "Qads/DockAreaWidget.h"
#include "Qads/DockAreaTitleBar.h"
#include "Qads/DockAreaTabBar.h"
#include "Qads/FloatingDockContainer.h"
#include "Qads/DockComponentsFactory.h"
#include "PlotJuggler/plotdata.h"

class DockWidget: public ads::CDockWidget
{
public:
  DockWidget(PlotDataMapRef& datamap, QWidget* parent = nullptr);
};

class PlotDocker: public ads::CDockManager
{
public:
  PlotDocker(PlotDataMapRef &datamap, QWidget* parent = nullptr);

private:


};

#endif // PLOT_DOCKER_H
