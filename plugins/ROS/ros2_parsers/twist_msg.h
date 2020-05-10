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

    TwistMsgParser(const std::string& topic_name):
        BuiltinMessageParser<geometry_msgs::msg::Twist>(topic_name)
    {
      _key.push_back(topic_name + "/linear/x");
      _key.push_back(topic_name + "/linear/y");
      _key.push_back(topic_name + "/linear/z");

      _key.push_back(topic_name + "/angular/x");
      _key.push_back(topic_name + "/angular/y");
      _key.push_back(topic_name + "/angular/z");
    }

    void parseMessageImpl(PlotDataMapRef& plot_data,
                          const geometry_msgs::msg::Twist& msg,
                          double timestamp) override
    {
        auto* series = &getSeries(plot_data, _key[0]);
        series->pushBack( {timestamp, msg.linear.x} );

        series = &getSeries(plot_data, _key[1]);
        series->pushBack( {timestamp, msg.linear.y} );

        series = &getSeries(plot_data, _key[2]);
        series->pushBack( {timestamp, msg.linear.z} );

        series = &getSeries(plot_data, _key[3]);
        series->pushBack( {timestamp, msg.angular.x} );

        series = &getSeries(plot_data, _key[4]);
        series->pushBack( {timestamp, msg.angular.y} );

        series = &getSeries(plot_data, _key[5]);
        series->pushBack( {timestamp, msg.angular.z} );
    }
private:
    std::vector<std::string> _key;
};

class TwistStampedMsgParser: public BuiltinMessageParser<geometry_msgs::msg::TwistStamped>
{
public:

    TwistStampedMsgParser(const std::string& topic_name):
        BuiltinMessageParser<geometry_msgs::msg::TwistStamped>(topic_name),
        _twist_parser(topic_name)
    {
      _key.push_back(topic_name + "/header/stamp/sec");
      _key.push_back(topic_name + "/header/stamp/nanosec");
    }

    void parseMessageImpl(PlotDataMapRef& plot_data,
                          const geometry_msgs::msg::TwistStamped& msg,
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

        _twist_parser.parseMessageImpl(plot_data, msg.twist, timestamp);
    }
private:
    TwistMsgParser _twist_parser;
    std::vector<std::string> _key;
};

class TwistCovarianceMsgParser: public BuiltinMessageParser<geometry_msgs::msg::TwistWithCovariance>
{
public:

    TwistCovarianceMsgParser(const std::string& topic_name):
        BuiltinMessageParser<geometry_msgs::msg::TwistWithCovariance>(topic_name),
        _twist_parser(topic_name)
    { }

    void parseMessageImpl(PlotDataMapRef& plot_data,
                          const geometry_msgs::msg::TwistWithCovariance& msg,
                          double timestamp) override
    {
        _twist_parser.parseMessageImpl(plot_data, msg.twist, timestamp);
        ParseCovariance(_topic_name, plot_data, msg.covariance, timestamp);
    }
private:
    TwistMsgParser _twist_parser;
};

