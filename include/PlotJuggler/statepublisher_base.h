#ifndef STATEPUBLISHER_TEMPLATE_H
#define STATEPUBLISHER_TEMPLATE_H

#include <QObject>
#include <QtPlugin>
#include <QMenu>
#include <QDomElement>
#include <functional>
#include "PlotJuggler/plotdata.h"
#include "PlotJuggler/pj_plugin.h"

class StatePublisher : public PlotJugglerPlugin
{
  Q_OBJECT
public:
  virtual bool enabled() const = 0;

  virtual void updateState(double current_time) = 0;

  virtual void play(double interval) = 0;

  virtual ~StatePublisher() = default;

  virtual QWidget* embeddedWidget()
  {
    return nullptr;
  }

  void setDataMap(const PlotDataMapRef* datamap)
  {
    _datamap = datamap;
  }

public slots:
  virtual void setEnabled(bool enabled) = 0;

signals:
  void closed();

protected:
  const PlotDataMapRef* _datamap;
};

QT_BEGIN_NAMESPACE

#define StatePublisher_iid "com.icarustechnology.PlotJuggler.StatePublisher"

Q_DECLARE_INTERFACE(StatePublisher, StatePublisher_iid)

QT_END_NAMESPACE

#endif
