#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include "covariance_util.h"
#include "quaternion_msg.h"
#include "ros2_parser.h"

using Ros2Introspection::BuiltinMessageParser;

class PoseMsgParser: public BuiltinMessageParser<geometry_msgs::msg::Pose>
{
public:

    PoseMsgParser(const std::string& topic_name):
        BuiltinMessageParser<geometry_msgs::msg::Pose>(topic_name),
        _quat_parser(topic_name + "/orientation")
    {
      _key.push_back(topic_name + "/position/x");
      _key.push_back(topic_name + "/position/y");
      _key.push_back(topic_name + "/position/z");
    }

    void parseMessageImpl(PlotDataMapRef& plot_data,
                          const geometry_msgs::msg::Pose& msg,
                          double timestamp) override
    {
        auto* series = &getSeries(plot_data, _key[0]);
        series->pushBack( {timestamp, msg.position.x} );

        series = &getSeries(plot_data, _key[1]);
        series->pushBack( {timestamp, msg.position.y} );

        series = &getSeries(plot_data, _key[2]);
        series->pushBack( {timestamp, msg.position.z} );

        //-----------------------
        _quat_parser.parseMessageImpl(plot_data,
                                      msg.orientation,
                                      timestamp);
    }
private:
    QuaternionMsgParser _quat_parser;
    std::vector<std::string> _key;
};

class PoseStampedMsgParser: public BuiltinMessageParser<geometry_msgs::msg::PoseStamped>
{
public:

    PoseStampedMsgParser(const std::string& topic_name):
        BuiltinMessageParser<geometry_msgs::msg::PoseStamped>(topic_name),
        _pose_parser(topic_name)
    {
      _key.push_back(topic_name + "/header/stamp/sec");
      _key.push_back(topic_name + "/header/stamp/nanosec");
    }

    void parseMessageImpl(PlotDataMapRef& plot_data,
                          const geometry_msgs::msg::PoseStamped& msg,
                          double timestamp) override
    {
        if( _use_header_stamp )
        {
            timestamp = static_cast<double>(msg.header.stamp.sec) +
                        static_cast<double>(msg.header.stamp.nanosec)*1e-9;
        }
        auto& stamp_sec_series = getSeries(plot_data, _key[0]);
        stamp_sec_series.pushBack( {timestamp, msg.header.stamp.sec} );

        auto& stamp_nsec_series = getSeries(plot_data, _key[1]);
        stamp_nsec_series.pushBack( {timestamp, msg.header.stamp.nanosec} );

        _pose_parser.parseMessageImpl(plot_data, msg.pose, timestamp);
    }
private:
    PoseMsgParser _pose_parser;
    std::vector<std::string> _key;
};


class PoseCovarianceMsgParser: public BuiltinMessageParser<geometry_msgs::msg::PoseWithCovariance>
{
public:

    PoseCovarianceMsgParser(const std::string& topic_name):
        BuiltinMessageParser<geometry_msgs::msg::PoseWithCovariance>(topic_name),
        _pose_parser(topic_name)
    { }

    void parseMessageImpl(PlotDataMapRef& plot_data,
                          const geometry_msgs::msg::PoseWithCovariance& msg,
                          double timestamp) override
    {
        _pose_parser.parseMessageImpl(plot_data, msg.pose, timestamp);
        ParseCovariance(_topic_name, plot_data, msg.covariance, timestamp);
    }
private:
    PoseMsgParser _pose_parser;
};

