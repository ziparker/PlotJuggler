#ifndef DATA_STREAMER_TEMPLATE_H
#define DATA_STREAMER_TEMPLATE_H

#include <mutex>
#include <unordered_set>
#include "PlotJuggler/plotdata.h"
#include "PlotJuggler/pj_plugin.h"
#include "PlotJuggler/messageparser_base.h"

namespace PJ {

/**
 * @brief The DataStreamer base class to create your own plugin.
 *
 * Important. To avoid problems with thread safety, it is important that ANY update to
 * dataMap(), which share its elements with the main application, is protected by the mutex()
 *
 * In particular the periodic updates.
 */
class DataStreamer : public PlotJugglerPlugin
{
  Q_OBJECT
public:

  virtual void setAvailableParsers(const std::unordered_map<std::string, MessageParserPtr>* parsers )
  {
    _loaded_parsers = parsers;
  }

  virtual bool start(QStringList*) = 0;

  virtual void shutdown() = 0;

  virtual bool isRunning() const = 0;

  virtual ~DataStreamer() = default;

  std::mutex& mutex()
  {
    return _mutex;
  }

  void setMaximumRange(double range);

  PlotDataMapRef& dataMap()
  {
    return _data_map;
  }

  const PlotDataMapRef& dataMap() const
  {
    return _data_map;
  }

signals:

  // Request to clear previous data
  void clearBuffers();

  // signal published periodically when there is new data
  void dataReceived();

  // Stopping a plugin from the "inside"
  void closed();

private:
  std::mutex _mutex;
  PlotDataMapRef _data_map;
  QAction* _start_streamer;
  const std::unordered_map<std::string, MessageParserPtr>* _loaded_parsers;
};

inline void DataStreamer::setMaximumRange(double range)
{
  std::lock_guard<std::mutex> lock(mutex());
  for (auto& it : dataMap().numeric)
  {
    it.second.setMaximumRangeX(range);
  }
  for (auto& it : dataMap().user_defined)
  {
    it.second.setMaximumRangeX(range);
  }
}

}


QT_BEGIN_NAMESPACE
#define DataStream_iid "facontidavide.PlotJuggler3.DataStreamer"
Q_DECLARE_INTERFACE(PJ::DataStreamer, DataStream_iid)
QT_END_NAMESPACE

#endif
