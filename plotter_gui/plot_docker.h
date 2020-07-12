#ifndef PLOT_DOCKER_H
#define PLOT_DOCKER_H

#include <QDomElement>
#include "Qads/DockManager.h"
#include "Qads/DockWidget.h"
#include "Qads/DockAreaWidget.h"
#include "Qads/DockAreaTitleBar.h"
#include "Qads/DockAreaTabBar.h"
#include "Qads/FloatingDockContainer.h"
#include "Qads/DockComponentsFactory.h"
#include "PlotJuggler/plotdata.h"
#include "plotwidget.h"
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
  Q_OBJECT

public:
  DockWidget(PlotDataMapRef& datamap, QWidget* parent = nullptr);

  PlotWidget* plotWidget()
  {
    return static_cast<PlotWidget*>( widget() );
  }
};

class PlotDocker: public ads::CDockManager
{

Q_OBJECT

public:
  PlotDocker(QString name, PlotDataMapRef &datamap, QWidget* parent = nullptr);

  QString name() const;

  void setName(QString name);

  QDomElement xmlSaveState(QDomDocument& doc) const;

  bool xmlLoadState(QDomElement& element);

  int plotCount() const;

  PlotWidget* plotAt(int index);

  void setHorizontalLink(bool enabled);

  void zoomOut();

  void replot();

signals:

  void plotWidgetAdded(PlotWidget*);

private:
  QString _name;

};

#endif // PLOT_DOCKER_H
