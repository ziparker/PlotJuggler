#ifndef ROS_MESSAGEPARSER_H
#define ROS_MESSAGEPARSER_H

#include "PlotJuggler/messageparser_base.h"
#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>
#include <std_msgs/Header.h>

class RosMessageParser : public MessageParser
{
public:

    void setUseHeaderStamp( bool use )
    {
        _use_header_stamp = use;
    }

protected:
    bool _use_header_stamp;
};

template <typename MainType, typename SubType, class ChildParser>
class RosMessageStampedParser: public RosMessageParser
{
public:
    RosMessageStampedParser(const char* child_prefix):
        _child_prefix(child_prefix)
    {
        _data.emplace_back( "/header/seq" );
        _data.emplace_back( "/header/stamp" );
    }


    const std::unordered_set<MessageKey>& getCompatibleMessageKeys() const override
    {
        static std::unordered_set<MessageKey> compatible_key =
        { ros::message_traits::MD5Sum<MainType>::value() };
        return compatible_key;
    }

    virtual void pushRawMessage(const MessageKey& key, const RawMessage& buffer, double timestamp) override
    {
        std_msgs::Header header;
        ros::serialization::IStream is( const_cast<uint8_t*>(buffer.data()), buffer.size() );
        ros::serialization::deserialize(is, header);

        int header_size = sizeof( ros::Time ) + 8 + header.frame_id.size();

        _data[0].pushBack( {timestamp, header.seq} );
        _data[1].pushBack( {timestamp, header.stamp.toSec()} );

        RawMessage sub_buffer = buffer.subspan(header_size);

        _child_parser.pushRawMessage( key, sub_buffer, timestamp );
    }

    void extractData(PlotDataMapRef& plot_map, const std::string& prefix) override
    {
        for (auto& it: _data)
        {
            MessageParser::appendData(plot_map, prefix + it.name(), it);
        }
        _child_parser.extractData(plot_map, prefix + _child_prefix);
    }
private:
    std::vector<PlotData> _data;
    ChildParser _child_parser;
    std::string _child_prefix;
};

#endif // ROS_MESSAGEPARSER_H
