#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include "quaternion_msg.h"
#include "ros2_parser.h"

using Ros2Introspection::BuiltinMessageParser;

class TwistMsgParser: public BuiltinMessageParser<geometry_msgs::msg::Twist>
{
public:

    TwistMsgParser(): BuiltinMessageParser<geometry_msgs::msg::Twist>()
    { }

    void parseMessageImpl(const std::string& topic_name,
                          PlotDataMapRef& plot_data,
                          const geometry_msgs::msg::Twist& msg,
                          double timestamp) override
    {
        auto* series = &getSeries(plot_data, topic_name + "/linear/x");
        series->pushBack( {timestamp, msg.linear.x} );

        series = &getSeries(plot_data, topic_name + "/linear/y");
        series->pushBack( {timestamp, msg.linear.y} );

        series = &getSeries(plot_data, topic_name + "/linear/z");
        series->pushBack( {timestamp, msg.linear.z} );

        series = &getSeries(plot_data, topic_name + "/angular/x");
        series->pushBack( {timestamp, msg.angular.x} );

        series = &getSeries(plot_data, topic_name + "/angular/y");
        series->pushBack( {timestamp, msg.angular.y} );

        series = &getSeries(plot_data, topic_name + "/angular/z");
        series->pushBack( {timestamp, msg.angular.z} );
    }
private:

};

class TwistStampedMsgParser: public BuiltinMessageParser<geometry_msgs::msg::TwistStamped>
{
public:

    TwistStampedMsgParser(): BuiltinMessageParser<geometry_msgs::msg::TwistStamped>()
    { }

    void parseMessageImpl(const std::string& topic_name,
                          PlotDataMapRef& plot_data,
                          const geometry_msgs::msg::TwistStamped& msg,
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

        _twist_parser.parseMessageImpl(topic_name, plot_data, msg.twist, timestamp);
    }
private:
    TwistMsgParser _twist_parser;
};

class TwistCovarianceMsgParser: public BuiltinMessageParser<geometry_msgs::msg::TwistWithCovariance>
{
public:

    TwistCovarianceMsgParser(): BuiltinMessageParser<geometry_msgs::msg::TwistWithCovariance>()
    { }

    void parseMessageImpl(const std::string& topic_name,
                          PlotDataMapRef& plot_data,
                          const geometry_msgs::msg::TwistWithCovariance& msg,
                          double timestamp) override
    {
        _twist_parser.parseMessageImpl(topic_name, plot_data, msg.twist, timestamp);
        ParseCovariance(topic_name, plot_data, msg.covariance, timestamp);
    }
private:
    TwistMsgParser _twist_parser;
};

