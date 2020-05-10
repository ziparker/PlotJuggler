#pragma once

#include <sensor_msgs/msg/joint_state.hpp>
#include "ros2_parser.h"

using Ros2Introspection::BuiltinMessageParser;

class JointStateMsgParser: public BuiltinMessageParser<sensor_msgs::msg::JointState>
{
public:

    JointStateMsgParser(const std::string& topic_name):
      BuiltinMessageParser<sensor_msgs::msg::JointState>(topic_name)
    {
      _key.push_back(topic_name + "/header/stamp/sec");
      _key.push_back(topic_name + "/header/stamp/nanosec");
    }

    void parseMessageImpl(PlotDataMapRef& plot_data,
                          const sensor_msgs::msg::JointState& msg,
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

      for(int i=0; i < msg.name.size(); i++)
      {
        const std::string prefix = _topic_name + "/" +  msg.name[i];

        if( msg.name.size() == msg.position.size())
        {
          auto& series = getSeries(plot_data, prefix + "/position" );
          series.pushBack( {timestamp, msg.position[i]} );
        }

        if( msg.name.size() == msg.velocity.size())
        {
          auto& series = getSeries(plot_data, prefix + "/velocity" );
          series.pushBack( {timestamp, msg.velocity[i]} );
        }

        if( msg.name.size() == msg.effort.size())
        {
          auto& series = getSeries(plot_data, prefix + "/effort" );
          series.pushBack( {timestamp, msg.effort[i]} );
        }
      }
    }
private:
    std::vector<std::string> _key;
};

