#pragma once

#include <geometry_msgs/msg/quaternion.hpp>
#include "ros2_parser.h"

using Ros2Introspection::BuiltinMessageParser;

class QuaternionMsgParser: public BuiltinMessageParser<geometry_msgs::msg::Quaternion>
{
public:

    QuaternionMsgParser(): BuiltinMessageParser<geometry_msgs::msg::Quaternion>()
    { }

    void parseMessageImpl(const std::string& topic_name,
                          PlotDataMapRef& plot_data,
                          const geometry_msgs::msg::Quaternion& msg,
                          double timestamp) override
    {
        auto* series = &getSeries(plot_data, topic_name + "/x");
        series->pushBack( {timestamp, msg.x} );

        series = &getSeries(plot_data, topic_name + "/y");
        series->pushBack( {timestamp, msg.y} );

        series = &getSeries(plot_data, topic_name + "/z");
        series->pushBack( {timestamp, msg.z} );

        series = &getSeries(plot_data, topic_name + "/w");
        series->pushBack( {timestamp, msg.w} );

        //-----------------------------
        auto q = msg;
        double quat_norm2 = q.w*q.w + q.x*q.x * q.y*q.y + q.z*q.z;
        if( std::abs(quat_norm2-1.0) > std::numeric_limits<double>::epsilon())
        {
            double mult = 1.0 / std::sqrt( quat_norm2 );
            q.x *= mult;
            q.y *= mult;
            q.z *= mult;
            q.w *= mult;
        }

        double roll, pitch, yaw;
        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (q.w * q.y - q.z * q.x);
        if (std::abs(sinp) >= 1) {
            pitch = std::copysign(M_PI_2, sinp); // use 90 degrees if out of range
        } else {
            pitch = std::asin(sinp);
        }

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        yaw = std::atan2(siny_cosp, cosy_cosp);

        const double RAD_TO_DEG = 180.0 / M_PI;

        series = &getSeries(plot_data, topic_name + "/roll_deg");
        series->pushBack({timestamp, RAD_TO_DEG * roll});

        series = &getSeries(plot_data, topic_name + "/pitch_deg");
        series->pushBack({timestamp, RAD_TO_DEG * pitch});

        series = &getSeries(plot_data, topic_name + "/yaw_deg");
        series->pushBack({timestamp, RAD_TO_DEG * yaw});
    }
};

