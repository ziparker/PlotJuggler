#pragma once

#include <string>
#include <unordered_map>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rmw/rmw.h"
#include "rmw/types.h"
#include "rosbag2/typesupport_helpers.hpp"
#include "rosbag2/types/introspection_message.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "ros2_introspection/ros2_introspection.hpp"
#include "PlotJuggler/plotdata.h"

//----------------------------------

namespace Ros2Introspection {

class MessageParserBase
{
public:
  MessageParserBase():
    _use_header_stamp(false)
  { }

  virtual ~MessageParserBase() = default;

  virtual void setUseHeaderStamp(bool use);

  virtual void setMaxArrayPolicy(MaxArrayPolicy discard_policy,
                                 size_t max_size) {}

  virtual bool parseMessage(const std::string& topic_name,
                            PlotDataMapRef& plot_data,
                            const rcutils_uint8_array_t *serialized_msg,
                            double timestamp) = 0;

  static PlotData& getSeries(PlotDataMapRef& plot_data, const std::string key);

  virtual const rosidl_message_type_support_t* typeSupport() const = 0;

protected:
  bool _use_header_stamp;
};

template <typename T>
class BuiltinMessageParser : public MessageParserBase
{
public:

  BuiltinMessageParser()
  {
    _type_support = rosidl_typesupport_cpp::get_message_type_support_handle<T>();
  }

  virtual bool parseMessage(const std::string& topic_name,
                            PlotDataMapRef& plot_data,
                            const rcutils_uint8_array_t *serialized_msg,
                            double timestamp) override
  {
    T msg;
    if ( RMW_RET_OK !=  rmw_deserialize(serialized_msg, _type_support, &msg))
    {
      throw std::runtime_error("failed to deserialize message");
    }
    parseMessageImpl(topic_name, plot_data, msg, timestamp);
    return true;
  }

  virtual void parseMessageImpl(const std::string& topic_name,
                                PlotDataMapRef& plot_data,
                                const T& msg,
                                double timestamp) = 0;

  const rosidl_message_type_support_t* typeSupport() const override
  {
    return _type_support;
  }
private:
  const rosidl_message_type_support_t* _type_support;
};


class IntrospectionParser: public MessageParserBase
{
public:
  IntrospectionParser(const std::string& topic_name, const std::string& topic_type):
    _intropection_parser(topic_name, topic_type)
  {}

  void setMaxArrayPolicy(MaxArrayPolicy discard_policy, size_t max_size) override;

  virtual bool parseMessage(const std::string& topic_name,
                            PlotDataMapRef& plot_data,
                            const rcutils_uint8_array_t *serialized_msg,
                            double timestamp) override;

  const rosidl_message_type_support_t* typeSupport() const override
  {
    return _intropection_parser.topicInfo().type_support;
  }

private:
  Ros2Introspection::Parser _intropection_parser;
  Ros2Introspection::FlatMessage _flat_msg;
  Ros2Introspection::RenamedValues _renamed;
};


class CompositeParser: public MessageParserBase
{
public:
  CompositeParser();

  virtual void setUseHeaderStamp(bool use) override;

  virtual void setMaxArrayPolicy(MaxArrayPolicy discard_policy,
                                 size_t max_size) override;

  void registerMessageType(const std::string& topic_name,
                           const std::string& topic_type);

  virtual bool parseMessage(const std::string& topic_name,
                            PlotDataMapRef& plot_data,
                            const rcutils_uint8_array_t *serialized_msg,
                            double timestamp) override;

  const rosidl_message_type_support_t* typeSupport() const override { return nullptr; }

  const rosidl_message_type_support_t* typeSupport(const std::string& topic_name) const;

private:
  std::unordered_map<std::string, std::shared_ptr<MessageParserBase>> _parsers;

  MaxArrayPolicy _discard_policy;

  size_t _max_array_size;
};

}


