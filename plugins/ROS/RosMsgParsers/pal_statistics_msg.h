#ifndef PAL_STATISTICS_MSG_H
#define PAL_STATISTICS_MSG_H

#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include "ros_parser_base.h"
#include <std_msgs/Header.h>
#include <absl/strings/str_cat.h>
#include <absl/strings/charconv.h>

struct PalStatisticsNames_
{
//    Header header
//    string[] names
//    uint32 names_version #This is increased each time names change
    std_msgs::Header header;
    std::vector<std::string> names;
    uint32_t names_version;
};

struct PalStatisticsValues_
{
//    Header header
//    float64[] names
//    uint32 names_version #This is increased each time names change
    std_msgs::Header header;
    std::vector<double> values;
    uint32_t names_version;
};

//-----------------------------------------------------

namespace ros
{
namespace serialization
{

template<> struct Serializer< ::PalStatisticsNames_ >
{
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
        stream.next(m.header);
        stream.next(m.names);
        stream.next(m.names_version);
    }
    ROS_DECLARE_ALLINONE_SERIALIZER
};

template<> struct Serializer< ::PalStatisticsValues_ >
{
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
        stream.next(m.header);
        stream.next(m.values);
        stream.next(m.names_version);
    }
    ROS_DECLARE_ALLINONE_SERIALIZER
};

} // namespace serialization
} // namespace ros

//-----------------------------------------------------

static std::unordered_map<uint32_t, std::vector<std::string> > _stored_pal_statistics_names;

class PalStatisticsNamesParser: public RosParserBase
{
public:

    PalStatisticsNamesParser() = default;

    static const std::string& getCompatibleKey()
    {
        static std::string temp =  "bece3d42a81d5c50cd68f110cf17bf55";
        return temp;
    }

    const std::unordered_set<std::string>& getCompatibleKeys() const override
    {
        static std::unordered_set<std::string> temp = { getCompatibleKey() };
        return temp;
    }

    virtual void pushRawMessage(const std::string& ,
                                const RawMessage& msg,
                                double) override
    {
        PalStatisticsNames_ pal_names;
        ros::serialization::IStream is( const_cast<uint8_t*>(msg.data()), msg.size() );
        ros::serialization::deserialize(is, pal_names);
        _stored_pal_statistics_names.insert( {pal_names.names_version, std::move(pal_names.names) });
    }

    void extractData(PlotDataMapRef& , const std::string& ) override
    {  }
};

//-----------------------------------------------------
class PalStatisticsValuesParser: public RosParserBase
{
public:

    PalStatisticsValuesParser() = default;

    static const std::string& getCompatibleKey()
    {
        static std::string temp =  "44646896ace86f96c24fbb63054eeee8";
        return temp;
    }

    const std::unordered_set<std::string>& getCompatibleKeys() const override
    {
        static std::unordered_set<std::string> temp = { getCompatibleKey() };
        return temp;
    }

    virtual void pushRawMessage(const std::string& ,
                                const RawMessage& msg,
                                double timestamp) override
    {
        PalStatisticsValues_ pal_values;
        ros::serialization::IStream is( const_cast<uint8_t*>(msg.data()), msg.size() );
        ros::serialization::deserialize(is, pal_values);

        const auto& names = _stored_pal_statistics_names[ pal_values.names_version ];
        const auto& values = pal_values.values;

        if( _use_header_stamp )
        {
            timestamp = pal_values.header.stamp.toSec();
        }

        for( size_t index = 0; index < values.size(); index++)
        {
            if( index >= names.size() ) break;

            const auto &key = names[index] ;

            auto data_it = _data.find( key );
            if( data_it == _data.end() )
            {
                data_it = _data.emplace( std::piecewise_construct,
                                         std::forward_as_tuple(key),
                                         std::forward_as_tuple(key)
                                         ).first;
            }
            data_it->second.pushBack( { timestamp, values[index] } );
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



#endif // PAL_STATISTICS_MSG_H
