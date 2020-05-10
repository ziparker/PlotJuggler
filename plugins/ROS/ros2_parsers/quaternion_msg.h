#pragma once

#include <geometry_msgs/msg/quaternion.hpp>
#include "ros2_parser.h"

using Ros2Introspection::BuiltinMessageParser;

class QuaternionMsgParser: public BuiltinMessageParser<geometry_msgs::msg::Quaternion>
{
public:

    QuaternionMsgParser(const std::string& topic_name):
        BuiltinMessageParser<geometry_msgs::msg::Quaternion>(topic_name)
    {
      _key.push_back(topic_name + "/x");
      _key.push_back(topic_name + "/y");
      _key.push_back(topic_name + "/z");
      _key.push_back(topic_name + "/w");

      _key.push_back(topic_name + "/roll_deg");
      _key.push_back(topic_name + "/pitch_deg");
      _key.push_back(topic_name + "/yaw_deg");
    }

    void parseMessageImpl(PlotDataMapRef& plot_data,
                          const geometry_msgs::msg::Quaternion& msg,
                          double timestamp) override
    {
        auto* series = &getSeries(plot_data, _key[0]);
        series->pushBack( {timestamp, msg.x} );

        series = &getSeries(plot_data, _key[1]);
        series->pushBack( {timestamp, msg.y} );

        series = &getSeries(plot_data, _key[2]);
        series->pushBack( {timestamp, msg.z} );

        series = &getSeries(plot_data, _key[3]);
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

        series = &getSeries(plot_data, _key[4]);
        series->pushBack({timestamp, RAD_TO_DEG * roll});

        series = &getSeries(plot_data, _key[5]);
        series->pushBack({timestamp, RAD_TO_DEG * pitch});

        series = &getSeries(plot_data, _key[6]);
        series->pushBack({timestamp, RAD_TO_DEG * yaw});
    }
private:
    std::vector<std::string> _key;
};

