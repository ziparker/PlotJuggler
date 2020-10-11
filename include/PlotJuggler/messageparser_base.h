#pragma once

#include <QtPlugin>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <map>
#include <set>
#include "PlotJuggler/plotdata.h"

class MessageRef
{
public:
  explicit MessageRef(uint8_t* first_ptr, size_t size) : _first_ptr(first_ptr), _size(size)
  {
  }

  explicit MessageRef(std::vector<uint8_t>& vect) : _first_ptr(vect.data()), _size(vect.size())
  {
  }

  const uint8_t* data() const {
    return _first_ptr;
  }

  uint8_t* data() {
    return _first_ptr;
  }

  size_t size() const {
    return _size;
  }

private:
  uint8_t* _first_ptr;
  size_t _size;
};

/**
 * @brief The MessageParser is the base class to create plugins that are able to parse one or
 * multiple Message types.
 * Each message type is uniquely identified by a MessageKey (128 bits, sufficiently large to
 * hold a MD5Sum identifier).
 *
 * You push one or more raw messages using the method pushMessageRef()
 * Once you have done, the result can be copied using plotData()
 */
class MessageParser
{
public:
  MessageParser(): _plot_data(nullptr)  {

  }
  virtual ~MessageParser() = default;

  void init(const std::string& topic_name, PlotDataMapRef* plot_data)
  {
    _topic_name = topic_name;
    _plot_data = plot_data;
  }

  virtual const std::string& formatName() const = 0;

  virtual bool parseMessage(const MessageRef serialized_msg, double timestamp) = 0;

protected:

  std::string _topic_name;
  PlotDataMapRef* _plot_data;

  PlotData& getSeries(const std::string& key)
  {
    auto plot_pair = _plot_data->numeric.find(key);
    if (plot_pair == _plot_data->numeric.end())
    {
      plot_pair = _plot_data->addNumeric(key);
    }
    return plot_pair->second;
  }
};

QT_BEGIN_NAMESPACE

#define MessageParser_iid "com.icarustechnology.PlotJuggler.MessageParser"
Q_DECLARE_INTERFACE(MessageParser, MessageParser_iid)

QT_END_NAMESPACE


using MessageParserPtr = std::shared_ptr<MessageParser>;

class MessageParserFactory
{
public:

private:
  MessageParserFactory() {}
  MessageParserFactory(const MessageParserFactory&) = delete;
  MessageParserFactory& operator=(const MessageParserFactory&) = delete;

  std::map<std::string, std::function<MessageParserPtr()>> creators_;
  std::set<std::string> names_;

  static MessageParserFactory& get()
  {
    static MessageParserFactory instance_;
    return instance_;
  }

public:

  static const std::set<std::string>& registeredFormats()
  {
    return get().names_;
  }

  template <typename T> static void registerParser()
  {
    T temp;
    std::string name = temp.formatName();
    get().names_.insert(name);
    get().creators_[name] = [](){ return std::make_shared<T>(); };
  }

  static MessageParserPtr create(const std::string& name,
                                 const std::string& topic_name,
                                 PlotDataMapRef& plot_data )
  {
    auto it = get().creators_.find(name);
    if( it == get().creators_.end())
    {
      return {};
    }
    auto creator = it->second();
    creator->init(topic_name, &plot_data);
    return creator;
  }
};

