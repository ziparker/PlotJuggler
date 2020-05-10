#pragma once

#include <sensor_msgs/msg/imu.hpp>
#include "fmt/format.h"
#include "ros2_parser.h"
#include "covariance_util.h"
#include "quaternion_msg.h"

using Ros2Introspection::BuiltinMessageParser;

class ImuMsgParser : public BuiltinMessageParser<sensor_msgs::msg::Imu>
{
public:
    ImuMsgParser(const std::string& topic_name)
        : BuiltinMessageParser<sensor_msgs::msg::Imu>(topic_name),
        _quat_parser(topic_name + "/orientation")
    {
      _key.push_back(topic_name + "/header/stamp/sec");
      _key.push_back(topic_name + "/header/stamp/nanosec");

      _key.push_back(topic_name + "/angular_velocity/x");
      _key.push_back(topic_name + "/angular_velocity/y");
      _key.push_back(topic_name + "/angular_velocity/z");

      _key.push_back(topic_name + "/linear_acceleration/x");
      _key.push_back(topic_name + "/linear_acceleration/y");
      _key.push_back(topic_name + "/linear_acceleration/z");
    }

    void parseMessageImpl(PlotDataMapRef &plot_data,
                          const sensor_msgs::msg::Imu &msg,
                          double timestamp) override
    {
        if (_use_header_stamp) {
            timestamp = static_cast<double>(msg.header.stamp.sec)
                        + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
        }

        auto *series = &getSeries(plot_data, _key[0]);
        series->pushBack({timestamp, msg.header.stamp.sec});

        series = &getSeries(plot_data, _key[1]);
        series->pushBack({timestamp, msg.header.stamp.sec});

        //--------------------
        series = &getSeries(plot_data, _key[2]);
        series->pushBack({timestamp, msg.angular_velocity.x});

        series = &getSeries(plot_data, _key[3]);
        series->pushBack({timestamp, msg.angular_velocity.y});

        series = &getSeries(plot_data, _key[4]);
        series->pushBack({timestamp, msg.angular_velocity.z});

        //--------------------
        series = &getSeries(plot_data, _key[5]);
        series->pushBack({timestamp, msg.linear_acceleration.x});

        series = &getSeries(plot_data, _key[6]);
        series->pushBack({timestamp, msg.linear_acceleration.y});

        series = &getSeries(plot_data, _key[7]);
        series->pushBack({timestamp, msg.linear_acceleration.z});

        //--------------------
        _quat_parser.parseMessageImpl(plot_data, msg.orientation, timestamp);

        ParseCovariance(_topic_name + "/orientation_covariance",
                        plot_data,msg.orientation_covariance, timestamp);

        ParseCovariance(_topic_name + "/linear_acceleration_covariance",
                        plot_data, msg.linear_acceleration_covariance, timestamp);

        ParseCovariance(_topic_name + "/angular_velocity_covariance",
                        plot_data, msg.angular_velocity_covariance, timestamp);
    }
private:
    QuaternionMsgParser _quat_parser;
    std::vector<std::string> _key;
};
