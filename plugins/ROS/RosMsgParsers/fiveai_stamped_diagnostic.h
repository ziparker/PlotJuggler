#ifndef FIVEAI_STAMPED_DIAGNOSTIC_H
#define FIVEAI_STAMPED_DIAGNOSTIC_H

#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


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



#endif // FIVEAI_STAMPED_DIAGNOSTIC_H
