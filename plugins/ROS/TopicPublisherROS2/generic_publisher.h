#ifndef GENERIC_PUBLISHER_H
#define GENERIC_PUBLISHER_H

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

class GenericPublisher : public rclcpp::PublisherBase
{
public:
  GenericPublisher(rclcpp::node_interfaces::NodeBaseInterface* node_base,
                   const rosidl_message_type_support_t& type_support, const std::string& topic_name,
                   const rclcpp::QoS& qos)
    : rclcpp::PublisherBase(node_base, topic_name, type_support, get_publisher_options(qos))
  {
  }

  virtual ~GenericPublisher() = default;

  void publish(std::shared_ptr<rmw_serialized_message_t> message)
  {
    auto return_code = rcl_publish_serialized_message(get_publisher_handle(), message.get(), NULL);

    if (return_code != RCL_RET_OK)
    {
      rclcpp::exceptions::throw_from_rcl_error(return_code, "failed to publish serialized message");
    }
  }
private:
  inline rcl_publisher_options_t get_publisher_options(const rclcpp::QoS& qos)
  {
    auto options = rcl_publisher_get_default_options();
    options.qos = qos.get_rmw_qos_profile();
    return options;
  }

};




#endif  // GENERIC_PUBLISHER_H
