#pragma once

#include <string>
#include <unordered_map>
#include <memory>
#include <vector>

#include "std_msgs/msg/header.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/rmw.h"
#include "rmw/types.h"
#include "rosbag2/typesupport_helpers.hpp"
#include "rosbag2/types/introspection_message.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "ros2_introspection/ros2_introspection.hpp"

#include "PlotJuggler/messageparser_base.h"

struct TopicInfo{
    TopicInfo(const std::string& type)
    {
        topic_type = type;
        const auto typesupport_identifier   = rosidl_typesupport_cpp::typesupport_identifier;
        const auto introspection_identifier = rosidl_typesupport_introspection_cpp::typesupport_identifier;

        introspection_support = rosbag2::get_typesupport(type, introspection_identifier);
        type_support = rosbag2::get_typesupport(type, typesupport_identifier);
        has_header_stamp = Ros2Introspection::TypeHasHeader( introspection_support );
        buffer = rosbag2::allocate_introspection_message(introspection_support, &allocator);
    }

    std::string topic_type;
    bool has_header_stamp;
    std::shared_ptr<rosbag2_introspection_message_t> buffer;
    const rosidl_message_type_support_t *introspection_support;
    const rosidl_message_type_support_t *type_support;
    Ros2Introspection::FlatMessage flat_msg;
    Ros2Introspection::RenamedValues renamed;

    static rcutils_allocator_t allocator;
};

//----------------------------------

template <typename T>
class Ros2MessageParser : public MessageParser
{
public:
    Ros2MessageParser():
    _use_header_stamp(false)
    { }

    const std::unordered_set<std::string>& getCompatibleKeys() const override
    {
      static std::unordered_set<std::string> temp = {
        rosidl_generator_traits::data_type<T>() };
      return temp;
    }

    virtual void pushMessageRef(const std::string&,
                                MessageRef& serialized_msg,
                                double timestamp) {

      rmw_serialized_message_t rwt_msg;
      rwt_msg.buffer = const_cast<uint8_t*>(serialized_msg.data());
      rwt_msg.buffer_length = serialized_msg.size();
      T msg;
      auto type_support = rosidl_typesupport_cpp::get_message_type_support_handle<T>();
      int ret =rmw_deserialize(&rwt_msg, type_support, &msg);
      if (ret != RMW_RET_OK) {
        throw std::runtime_error("failed to deserialize message");
      }
      parseMessage(msg, timestamp);
    }

    virtual void parseMessage(const T& msg,
                              double timestamp) = 0;

    virtual void setUseHeaderStamp( bool use )
    {
        _use_header_stamp = use;
    }

protected:
    bool _use_header_stamp;
};




