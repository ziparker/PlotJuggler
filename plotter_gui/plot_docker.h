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

#include "ui_plot_docker_toolbar.h"

class DraggableToolbar : public QWidget
{
  Q_OBJECT

public:
  explicit DraggableToolbar(ads::CDockWidget* parent);
  ~DraggableToolbar();

  QLabel* label(){ return ui->label;}
  QPushButton* buttonSplitHorizontal() { return ui->buttonSplitHorizontal; }
  QPushButton* buttonSplitVertical() { return ui->buttonSplitVertical; }
  QPushButton* buttonFullscreen() { return ui->buttonFullscreen; }
  QPushButton* buttonEdit() { return ui->buttonEdit; }
  QPushButton* buttonZoomOut() { return ui->buttonZoomOut; }
  QPushButton* buttonClose() { return ui->buttonClose; }

private:
  void mousePressEvent(QMouseEvent* ev) override;
  void mouseReleaseEvent(QMouseEvent* ev) override;
  void mouseMoveEvent(QMouseEvent* ev) override;
  void enterEvent(QEvent *) override;
  void leaveEvent(QEvent *) override;

  ads::CDockWidget* parent_;
  Ui::DraggableToolbar *ui;
  bool displayed_toolbar_;

  void showToolButtons(bool show);
};

class DockWidget: public ads::CDockWidget
{
public:
  DockWidget(PlotDataMapRef& datamap, QWidget* parent = nullptr);
private:

};

class PlotDocker: public ads::CDockManager
{
public:
  PlotDocker(PlotDataMapRef &datamap, QWidget* parent = nullptr);

private:


};

#endif // PLOT_DOCKER_H
