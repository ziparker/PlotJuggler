#ifndef GEOMETRY_MSG_TWIST_H
#define GEOMETRY_MSG_TWIST_H

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "ros_messageparser.h"

class GeometryMsgTwist: public RosMessageParser<geometry_msgs::Twist>
{
public:

    GeometryMsgTwist()
    {
        _data.emplace_back( "/linear/x" );
        _data.emplace_back( "/linear/y" );
        _data.emplace_back( "/linear/z" );
        _data.emplace_back( "/angular/x" );
        _data.emplace_back( "/angular/y" );
        _data.emplace_back( "/angular/z" );
    }

    virtual void pushRawMessage(const MessageKey& ,
                                const RawMessage& msg,
                                double timestamp) override
    {
        geometry_msgs::Twist twist;
        ros::serialization::IStream is( const_cast<uint8_t*>(msg.data()), msg.size() );
        ros::serialization::deserialize(is, twist);

        _data[0].pushBack( {timestamp, twist.linear.x} );
        _data[1].pushBack( {timestamp, twist.linear.y} );
        _data[2].pushBack( {timestamp, twist.linear.z} );
        _data[3].pushBack( {timestamp, twist.angular.x} );
        _data[4].pushBack( {timestamp, twist.angular.y} );
        _data[5].pushBack( {timestamp, twist.angular.z} );
    }

    void extractData(PlotDataMapRef& plot_map, const std::string& prefix) override
    {
        for (auto& it: _data)
        {
            appendData(plot_map, prefix + it.name(), it);
        }
    }

private:
    std::vector<PlotData> _data;
};

typedef
RosMessageStampedParser<geometry_msgs::TwistStamped, geometry_msgs::Twist, GeometryMsgTwist>
GeometryMsgTwistStamped;


#endif // GEOMETRY_MSG_TWIST_H
