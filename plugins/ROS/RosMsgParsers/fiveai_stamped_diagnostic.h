#ifndef FIVEAI_STAMPED_DIAGNOSTIC_H
#define FIVEAI_STAMPED_DIAGNOSTIC_H

#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>
#include "ros_messageparser.h"
#include <absl/strings/str_cat.h>
#include <absl/strings/charconv.h>

struct StampedDiagnostic_
{
    StampedDiagnostic_()
        : status(0)
        , stamp()
        , key()
        , value()  {
    }

    uint8_t status;
    ros::Time stamp;

    std::string key;
    std::string value;

}; // struct StampedDiagnostic_

struct NodeDiagnostics_
{
    std::vector< StampedDiagnostic_ > diagnostics;
};

//-----------------------------------------------------


namespace ros
{
namespace serialization
{

template<> struct Serializer< ::StampedDiagnostic_ >
{
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
        stream.next(m.status);
        stream.next(m.stamp);
        stream.next(m.key);
        stream.next(m.value);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct StampedDiagnostic_

template<> struct Serializer< ::NodeDiagnostics_ >
{
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
        stream.next(m.diagnostics);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct NodeDiagnostics_

} // namespace serialization
} // namespace ros

//-----------------------------------------------------


class FiveAiDiagnosticMsg: public RosParserBase
{
public:

    FiveAiDiagnosticMsg()
    {
    }

    static const char* getCompatibleKey() { return "b47994f5f7cab18367c65bedb56d7f75"; }

    const std::unordered_set<std::string>& getCompatibleKeys() const override
    {
        return { getCompatibleKey() };
    }

    virtual void pushRawMessage(const std::string& ,
                                const RawMessage& msg,
                                double timestamp) override
    {
        NodeDiagnostics_ diagnostic;
        ros::serialization::IStream is( const_cast<uint8_t*>(msg.data()), msg.size() );
        ros::serialization::deserialize(is, diagnostic);

        for( const auto& it: diagnostic.diagnostics)
        {
            if( _use_header_stamp )
            {
                timestamp = it.stamp.toSec();
            }
            const char *start_ptr = it.value.data();
            double val = 0;
            auto res = absl::from_chars (start_ptr, start_ptr + it.value.size(), val);

            auto data_it = _data.find( it.key );
            if( data_it == _data.end() )
            {
                data_it = _data.emplace( std::piecewise_construct,
                                         std::forward_as_tuple(it.key),
                                         std::forward_as_tuple(it.key)
                                         ).first;
            }
            data_it->second.pushBack( { timestamp, val } );
        }
    }

    void extractData(PlotDataMapRef& plot_map, const std::string& prefix) override
    {
        for (auto& it: _data)
        {
            appendData(plot_map,
                       absl::StrCat(prefix, "/", it.first),
                       it.second);
        }
    }

private:
    std::unordered_map<std::string,PlotData> _data;
};



#endif // FIVEAI_STAMPED_DIAGNOSTIC_H
