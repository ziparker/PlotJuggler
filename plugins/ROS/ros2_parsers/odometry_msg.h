#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include "pose_msg.h"
#include "twist_msg.h"
#include "ros2_parser.h"

using Ros2Introspection::BuiltinMessageParser;

class OdometryMsgParser: public BuiltinMessageParser<nav_msgs::msg::Odometry>
{
public:

    OdometryMsgParser(const std::string& topic_name):
        BuiltinMessageParser<nav_msgs::msg::Odometry>(topic_name),
        _pose_parser(topic_name + "/pose"),
        _twist_parser(topic_name + "/twist")
    {
      _key.push_back(topic_name + "/header/stamp/sec");
      _key.push_back(topic_name + "/header/stamp/nanosec");
    }

    void parseMessageImpl(PlotDataMapRef& plot_data,
                          const nav_msgs::msg::Odometry& msg,
                          double timestamp) override
    {
        if( _use_header_stamp )
        {
            timestamp = static_cast<double>(msg.header.stamp.sec) +
                        static_cast<double>(msg.header.stamp.nanosec)*1e-9;
        }

        auto* series = &getSeries(plot_data, _key[0]);
        series->pushBack( {timestamp, msg.header.stamp.sec} );

        series = &getSeries(plot_data, _key[1]);
        series->pushBack( {timestamp, msg.header.stamp.nanosec} );

        _pose_parser.parseMessageImpl(plot_data,  msg.pose, timestamp);
        _twist_parser.parseMessageImpl(plot_data,  msg.twist, timestamp);
    }

private:
    PoseCovarianceMsgParser  _pose_parser;
    TwistCovarianceMsgParser _twist_parser;
    std::vector<std::string> _key;
};

