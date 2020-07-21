#ifndef PLOT_DOCKER_H
#define PLOT_DOCKER_H

#include <QDomElement>
#include <QXmlStreamReader>
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
  QPushButton* buttonClose() { return ui->buttonClose; }

  void showToolButtons(bool show);

private:
  void mousePressEvent(QMouseEvent* ev) override;
  void mouseReleaseEvent(QMouseEvent* ev) override;
  void mouseMoveEvent(QMouseEvent* ev) override;
  void enterEvent(QEvent *) override;
  void leaveEvent(QEvent *) override;

  ads::CDockWidget* parent_;
  Ui::DraggableToolbar *ui;
  bool displayed_toolbar_;
};

class DockWidget: public ads::CDockWidget
{
  Q_OBJECT

public:
  DockWidget(PlotDataMapRef& datamap, QWidget* parent = nullptr);

  PlotWidget* plotWidget();

  DraggableToolbar* toolBar();

public slots:
  DockWidget* spliHorizontal();

  DockWidget* spliVertical();

private:
  DraggableToolbar* _toolbar;

  PlotDataMapRef& _datamap;

signals:
  void undoableChange();
};

class PlotDocker: public ads::CDockManager
{

Q_OBJECT

public:
  PlotDocker(QString name, PlotDataMapRef &datamap, QWidget* parent = nullptr);

  QString name() const;

  void setName(QString name);

  QDomElement xmlSaveState(QDomDocument& doc) const;

  bool xmlLoadState(QDomElement& tab_element);

  int plotCount() const;

  PlotWidget* plotAt(int index);

  void setHorizontalLink(bool enabled);

  void zoomOut();

  void replot();

private:

  void restoreSplitter(QDomElement elem, DockWidget* widget);

  QString _name;

  PlotDataMapRef& _datamap;

signals:

  void plotWidgetAdded(PlotWidget*);

  void undoableChange();

};

#endif // PLOT_DOCKER_H
