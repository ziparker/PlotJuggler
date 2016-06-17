#ifndef ROSTYPEPARSER_H
#define ROSTYPEPARSER_H

#include <ros/ros.h>
#include <rosbag/message_instance.h>
#include <vector>
#include <string>
#include <topic_tools/shape_shifter.h>

namespace RosFlatTypeParser{

typedef struct
{
    std::string basic_type_name;
    std::string field_name;
} FlatField;

typedef struct
{
    std::string ros_type_name;
    std::vector<FlatField> fields;
}FlatRosType;

class TopicFlatContainer{
public:
    FlatRosType type;
    std::vector< std::pair<std::string, std::int64_t> >  vect_integer;
    std::vector< std::pair<std::string, double > >       vect_double;
    std::vector< std::pair<std::string, ros::Time > >    vect_time;
    std::vector< std::pair<std::string, std::string > >  vect_string;
    int buffer_size;

    TopicFlatContainer() {
        buffer_size = 0;
    }
};


std::vector<FlatField> buildFlatTypeHierarchy(std::string prefix,
                                          const std::vector<FlatRosType>& types,
                                          const std::string& type_name);

std::vector<FlatRosType>  parseRosTypeDescription(
        const std::string & type_name,
        const std::string & msg_definition );

//--------------------------
std::ostream& operator<<(std::ostream& s, const TopicFlatContainer& c);

TopicFlatContainer buildFlatContainer(const std::string& msg_dataType,
                                 const std::string& msg_definition,
                                 uint8_t* msg_buffer,
                                 size_t buffer_size,
                                 const std::string &topic_name,
                                 int max_vector_size);

TopicFlatContainer buildFlatContainer(const topic_tools::ShapeShifter::ConstPtr& msg,
                                 const std::string& topic_name,
                                 int max_vector_size);

TopicFlatContainer buildFlatContainer(const rosbag::MessageInstance& msg,
                                 int max_vector_size);

//--------------------------

void extractFlatContainer(uint8_t* msg_buffer,
                          TopicFlatContainer* container,
                          int max_vector_size, bool canAppend = false);


}

#endif // ROSTYPEPARSER_H
