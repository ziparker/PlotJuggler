#pragma once

#include <sensor_msgs/msg/joint_state.hpp>
#include "ros2_parser.h"

class JointStateMsgParser: public Ros2MessageParser<sensor_msgs::msg::JointState>
{
public:

    JointStateMsgParser()
    { }

    void parseMessage(const sensor_msgs::msg::JointState& msg, double timestamp) override
    {
      for(int i=0; i < msg.name.size(); i++)
      {
        const auto& joint_name = msg.name[i];
        auto data_it = _data.find( joint_name );
        if( data_it == _data.end() ){
          data_it = _data.insert( {joint_name, joint_name} ).first;
        }
        data_it->second.position.pushBack( {timestamp, msg.position[i]} );
        data_it->second.velocity.pushBack( {timestamp, msg.velocity[i]} );
        data_it->second.effort.pushBack(   {timestamp, msg.effort[i]} );
      }
    }

    void extractData(PlotDataMapRef& plot_map, const std::string& prefix) override
    {
        for (auto& it: _data)
        {
          auto& data_joint = it.second;
          appendData(plot_map, prefix + data_joint.position.name(), data_joint.position);
          appendData(plot_map, prefix + data_joint.velocity.name(), data_joint.velocity);
          appendData(plot_map, prefix + data_joint.effort.name(),   data_joint.effort);
        }
    }

private:
    struct PlotDataJoint{
      PlotDataJoint( const std::string& joint_name):
        position(joint_name + "/position" ),
        velocity(joint_name + "/velocity" ),
        effort(joint_name + "/effort" )
      {

      }
      PlotData position;
      PlotData velocity;
      PlotData effort;
    };
    std::unordered_map<std::string, PlotDataJoint> _data;
};

