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

    PoseMsgParser(): BuiltinMessageParser<geometry_msgs::msg::Pose>()
    { }

    void parseMessageImpl(const std::string& topic_name,
                          PlotDataMapRef& plot_data,
                          const geometry_msgs::msg::Pose& msg,
                          double timestamp) override
    {
        auto* series = &getSeries(plot_data, topic_name + "/position/x");
        series->pushBack( {timestamp, msg.position.x} );

        series = &getSeries(plot_data, topic_name + "/position/y");
        series->pushBack( {timestamp, msg.position.y} );

        series = &getSeries(plot_data, topic_name + "/position/z");
        series->pushBack( {timestamp, msg.position.z} );

        //-----------------------
        _quat_parser.parseMessageImpl(topic_name + "/orientation",
                                      plot_data,
                                      msg.orientation,
                                      timestamp);
    }
private:
    QuaternionMsgParser _quat_parser;
};

class PoseStampedMsgParser: public BuiltinMessageParser<geometry_msgs::msg::PoseStamped>
{
public:

    PoseStampedMsgParser(): BuiltinMessageParser<geometry_msgs::msg::PoseStamped>()
    { }

    void parseMessageImpl(const std::string& topic_name,
                          PlotDataMapRef& plot_data,
                          const geometry_msgs::msg::PoseStamped& msg,
                          double timestamp) override
    {
        if( _use_header_stamp )
        {
            timestamp = static_cast<double>(msg.header.stamp.sec) +
                        static_cast<double>(msg.header.stamp.nanosec)*1e-9;
        }
        auto& stamp_sec_series = getSeries(plot_data, topic_name + "/header/stamp/sec");
        stamp_sec_series.pushBack( {timestamp, msg.header.stamp.sec} );

        auto& stamp_nsec_series = getSeries(plot_data, topic_name + "/header/stamp/nanosec");
        stamp_nsec_series.pushBack( {timestamp, msg.header.stamp.nanosec} );

        _pose_parser.parseMessageImpl(topic_name, plot_data, msg.pose, timestamp);
    }
private:
    PoseMsgParser _pose_parser;
};


class PoseCovarianceMsgParser: public BuiltinMessageParser<geometry_msgs::msg::PoseWithCovariance>
{
public:

    PoseCovarianceMsgParser(): BuiltinMessageParser<geometry_msgs::msg::PoseWithCovariance>()
    { }

    void parseMessageImpl(const std::string& topic_name,
                          PlotDataMapRef& plot_data,
                          const geometry_msgs::msg::PoseWithCovariance& msg,
                          double timestamp) override
    {
        _pose_parser.parseMessageImpl(topic_name, plot_data, msg.pose, timestamp);
        ParseCovariance(topic_name, plot_data, msg.covariance, timestamp);
    }
private:
    PoseMsgParser _pose_parser;
};

