#pragma once

#include <QtPlugin>
#include <QApplication>
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
  explicit MessageRef(uint8_t* first_ptr, size_t size) :
    _first_ptr(first_ptr), _size(size)
  { }

  explicit MessageRef(std::vector<uint8_t>& vect) :
    _first_ptr(vect.data()), _size(vect.size())
  { }

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
  MessageParser(): _plot_data(nullptr), _use_message_stamp(false)  {

  }
  virtual ~MessageParser() = default;

  virtual void setUseMessageStamp(bool use)
  {
    _use_message_stamp = use;
  }

  void init(const std::string& topic_name, PlotDataMapRef* plot_data)
  {
    _topic_name = topic_name;
    _plot_data = plot_data;
  }

  virtual const char* formatName() const = 0;

  virtual bool parseMessage(const MessageRef serialized_msg, double timestamp) = 0;

protected:

  std::string _topic_name;
  PlotDataMapRef* _plot_data;
  bool _use_message_stamp;

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

class MessageParserFactory: public QObject
{
private:
  MessageParserFactory(const MessageParserFactory&) = delete;
  MessageParserFactory& operator=(const MessageParserFactory&) = delete;

  std::map<std::string, std::function<MessageParserPtr()>> creators_;
  std::set<std::string> names_;

  static MessageParserFactory* instance();

public:
  MessageParserFactory() {}

  static const std::set<std::string>& registeredFormats() {
    return instance()->names_;
  }

  template <typename T> static void registerParser()
  {
    T temp;
    std::string name = temp.formatName();
    instance()->names_.insert(name);
    instance()->creators_[name] = [](){ return std::make_shared<T>(); };
  }

  static MessageParserPtr create(const std::string& name,
                                 const std::string& topic_name,
                                 PlotDataMapRef& plot_data )
  {
    auto it = instance()->creators_.find(name);
    if( it == instance()->creators_.end())
    {
      return {};
    }
    auto creator = it->second();
    creator->init(topic_name, &plot_data);
    return creator;
  }
};

Q_DECLARE_OPAQUE_POINTER(MessageParserFactory *)
Q_DECLARE_METATYPE(MessageParserFactory *)
Q_GLOBAL_STATIC(MessageParserFactory, _message_parser_ptr_from_macro)

inline MessageParserFactory* MessageParserFactory::instance()
{
  static MessageParserFactory * _ptr(nullptr);
  if (!qApp->property("MessageParserFactory").isValid() && !_ptr) {
    _ptr = _message_parser_ptr_from_macro;
    qApp->setProperty("MessageParserFactory", QVariant::fromValue(_ptr));
  }
  else if (!_ptr) {
    _ptr = qvariant_cast<MessageParserFactory *>(qApp->property("MessageParserFactory"));
  }
  else if (!qApp->property("MessageParserFactory").isValid()) {
    qApp->setProperty("MessageParserFactory", QVariant::fromValue(_ptr));
  }
  return _ptr;
}


