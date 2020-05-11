#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include "covariance_util.h"
#include "quaternion_msg.h"
#include "ros1_parser.h"

class PoseMsgParser: public BuiltinMessageParser<geometry_msgs::Pose>
{
public:

    PoseMsgParser(const std::string& topic_name, PlotDataMapRef& plot_data):
        BuiltinMessageParser<geometry_msgs::Pose>(topic_name, plot_data),
        _quat_parser(topic_name + "/orientation", plot_data)
    {
      _data.push_back( &getSeries(plot_data, topic_name + "/position/x") );
      _data.push_back( &getSeries(plot_data, topic_name + "/position/y") );
      _data.push_back( &getSeries(plot_data, topic_name + "/position/z") );
    }

    void parseMessageImpl(const geometry_msgs::Pose& msg,
                          double timestamp) override
    {
        _data[0]->pushBack( {timestamp, msg.position.x} );
        _data[1]->pushBack( {timestamp, msg.position.y} );
        _data[2]->pushBack( {timestamp, msg.position.z} );

        _quat_parser.parseMessageImpl( msg.orientation, timestamp);
    }
private:
    QuaternionMsgParser _quat_parser;
    std::vector<PlotData*> _data;
};

class PoseStampedMsgParser: public BuiltinMessageParser<geometry_msgs::PoseStamped>
{
public:

    PoseStampedMsgParser(const std::string& topic_name, PlotDataMapRef& plot_data):
        BuiltinMessageParser<geometry_msgs::PoseStamped>(topic_name, plot_data),
        _pose_parser(topic_name, plot_data)
    {
      _data.push_back( &getSeries(plot_data, topic_name + "/header/stamp/sec") );
      _data.push_back( &getSeries(plot_data, topic_name + "/header/stamp/nanosec") );
    }

    void parseMessageImpl(const geometry_msgs::PoseStamped& msg,
                          double timestamp) override
    {
        if( _use_header_stamp )
        {
            timestamp = double(msg.header.stamp.sec) +
                        double(msg.header.stamp.nsec)*1e-9;
        }
        _data[0]->pushBack( {timestamp, double(msg.header.stamp.sec)} );
        _data[1]->pushBack({timestamp,  double(msg.header.stamp.nsec)});

        _pose_parser.parseMessageImpl(msg.pose, timestamp);
    }
private:
    PoseMsgParser _pose_parser;
    std::vector<PlotData*> _data;
};


class PoseCovarianceMsgParser: public BuiltinMessageParser<geometry_msgs::PoseWithCovariance>
{
public:

    PoseCovarianceMsgParser(const std::string& topic_name, PlotDataMapRef& plot_data):
        BuiltinMessageParser<geometry_msgs::PoseWithCovariance>(topic_name, plot_data),
        _pose_parser(topic_name, plot_data),
        _covariance(topic_name + "/covariance", plot_data)
    { }

    void parseMessageImpl(const geometry_msgs::PoseWithCovariance& msg,
                          double timestamp) override
    {
        _pose_parser.parseMessageImpl(msg.pose, timestamp);
        _covariance.parse(msg.covariance, timestamp);
    }
private:
    PoseMsgParser _pose_parser;
    CovarianceParser<6> _covariance;
};

