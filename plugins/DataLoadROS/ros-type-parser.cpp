#include "ros-type-parser.h"
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <sstream>

namespace RosFlatTypeParser{

bool isCommentOrEmpty(const std::string& line)
{
    if(line.empty()) return true;

    int index = 0;
    while( index < line.size()-1 && line[index] == ' '  )
    {
        index++;
    }
    return (line[index] == '#');
}

bool isSeparator(const std::string& line)
{
    if(line.size() != 80 ) return false;
    for (int i=0; i<80; i++)
    {
        if( line[i] != '=') return false;
    }
    return true;
}


std::string stripTypeName( std::string line)
{
    for(int index = line.size() -1; index >=0; index--)
    {
        if( line[index] == '/')
        {
            line.erase(0,index+1);
            break;
        }
    }
    return line;
}


// recursive funtion
bool buildFlatTypeHierarchy_Impl(std::string prefix,
                                 const std::vector<FlatRosType>& types,
                                 const std::string& type_name,
                                 std::vector<FlatField>* output)
{
    int index = 0;
    for ( index = 0; index < types.size(); index++)
    {
        if( type_name.compare( types[index].ros_type_name ) == 0)
        {
            break;
        }
    }

    if( index >= types.size()) return false;

    const FlatRosType& type = types[index];

    for (int f=0; f< type.fields.size(); f++)
    {
        const FlatField& field = type.fields[f];
        std::string new_prefix ( prefix );
        new_prefix.append(".");
        new_prefix.append( field.field_name );

        if( !buildFlatTypeHierarchy_Impl( new_prefix, types, field.basic_type_name, output) )
        {
            FlatField new_field;
            new_field.field_name = new_prefix;
            new_field.basic_type_name  = field.basic_type_name;
            output->push_back( new_field );
        }
    }
    return true;
}

std::vector<FlatField> buildFlatTypeHierarchy(std::string prefix,
                                              const std::vector<FlatRosType>& types,
                                              const std::string& type_name)
{
    std::vector<FlatField> output;
    buildFlatTypeHierarchy_Impl( prefix, types, type_name, &output);
    return output;
}

std::vector<FlatRosType> parseRosTypeDescription(const std::string & type_name, const std::string & msg_definition)
{
    std::istringstream messageDescriptor(msg_definition);

    std::vector<FlatRosType> types;
    types.push_back( FlatRosType() );

    FlatRosType* current_type = &types.back();

    std::string main_typename = (type_name);

    current_type->ros_type_name = main_typename;

    for (std::string line; std::getline(messageDescriptor, line, '\n') ; )
    {
        if( isCommentOrEmpty(line) )
        {
            continue;
        }

        if( isSeparator(line) )
        {
            if( std::getline(messageDescriptor, line, '\n') == 0)
            {
                break;
            }

            types.push_back( FlatRosType() );
            current_type = &types.back();

            if( line.compare(0, 5, "MSG: ") == 0)
            {
                line.erase(0,5);
            }

            current_type->ros_type_name =  stripTypeName(line);
        }
        else{
            FlatField field;

            std::stringstream ss2(line);
            ss2 >> field.basic_type_name;
            ss2 >> field.field_name;

            current_type->fields.push_back( field );
        }
    }
    return types;
}

std::ostream &operator<<(std::ostream &s, const TopicFlatContainer &c)
{
    for (int i=0; i< c.vect_time.size(); i++ )
    {
        s << c.vect_time[i].first << " = " << c.vect_time[i].second << std::endl;
    }
    for (int i=0; i< c.vect_integer.size(); i++ )
    {
        s << c.vect_integer[i].first << " = " << c.vect_integer[i].second << std::endl;
    }
    for (int i=0; i< c.vect_double.size(); i++ )
    {
        s << c.vect_double[i].first << " = " << c.vect_double[i].second << std::endl;
    }
    for (int i=0; i< c.vect_string.size(); i++ )
    {
        s << c.vect_string[i].first << " = " << c.vect_string[i].second << std::endl;
    }
    return s;
}

template <typename T> T ReadFromBufferAndMoveForward( uint8_t** buffer)
{
    T destination =  (*( reinterpret_cast<T*>( *buffer ) ) );
    *buffer +=  sizeof(T);
    return destination;
}

TopicFlatContainer buildFlatContainer(const topic_tools::ShapeShifter::ConstPtr& msg,
                                      const std::string &topic_name, int max_vector_size)
{
    std::vector<uint8_t> buffer( msg->size() );
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    msg->write(stream);

    return buildFlatContainer( msg->getDataType(),
                               msg->getMessageDefinition(),
                               buffer.data(),
                               buffer.size(),
                               topic_name,
                               max_vector_size);
}

TopicFlatContainer buildFlatContainer(const rosbag::MessageInstance& msg,
                                      int max_vector_size)
{
    std::vector<uint8_t> buffer ( msg.size() );
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    msg.write(stream);

    return buildFlatContainer( msg.getDataType(),
                               msg.getMessageDefinition(),
                               buffer.data(),
                               buffer.size(),
                               msg.getTopic(),
                               max_vector_size);
}

TopicFlatContainer buildFlatContainer(const std::string& msg_dataType,
                                      const std::string& msg_definition,
                                      uint8_t* buffer_ptr,
                                      size_t buffer_size,
                                      const std::string& topic_name,
                                      int max_vector_size)
{
    TopicFlatContainer container;

    container.buffer_size = buffer_size;

    auto type_hierarchy = parseRosTypeDescription(
                msg_dataType,
                msg_definition   );

    auto flat_fields = buildFlatTypeHierarchy( topic_name.c_str(),
                                               type_hierarchy,
                                               msg_dataType );

    container.type.ros_type_name = msg_dataType;
    container.type.fields        = flat_fields;

    extractFlatContainer( buffer_ptr, &container, max_vector_size, true );

    return container;
}

template <typename T, typename V> void appendToVector(
        int &index,
        T& vector,
        const char* field_name,
        V& value,
        bool canGrow )
{
   /* std::cout << field_name << "  :   " << value
              << " / index: " << index
              << " /  vector size: " << vector.size() <<  std::endl;*/

    if( index <  vector.size()  ) {
        vector[ index++ ].second = value;
    }
    else if( canGrow ) {
        vector.push_back( std::make_pair( std::string(field_name), value ) );
        index++;
    }
}

void extractFlatContainer(uint8_t* buffer_ptr,
                          TopicFlatContainer* container,
                          int max_vector_size,
                          bool canAppend)
{
    int int_index    = 0;
    int float_index  = 0;
    int time_index   = 0;
    int string_index = 0;

    const std::string VECTOR_SYMBOL("[]");

    auto& flat_fields = container->type.fields;

    for (int i=0; i< flat_fields.size(); i++ )
    {
        std::string type = flat_fields[i].basic_type_name ;
        int32_t vect_size = 1;

        if( boost::contains(type, VECTOR_SYMBOL ))
        {
            vect_size = ReadFromBufferAndMoveForward<int32_t>( &buffer_ptr );
            type.erase( type.length() -2 );
        }

        if(vect_size > max_vector_size)
        {
            std::cout<< " skipping " <<  flat_fields[i].field_name << " size: " << vect_size << std::endl;
            // skip this
            continue;
        }
        else{
         //   std::cout<< " parsing " <<  flat_fields[i].field_name << " size: " << vect_size << std::endl;
        }


        for (int v=0; v < vect_size; v++)
        {
            char field_name[128];

            if( canAppend){
                if( vect_size > 1) {
                    sprintf(field_name, "%s.%d", flat_fields[i].field_name.c_str(), v);
                }
                else{
                    sprintf(field_name, "%s", flat_fields[i].field_name.c_str());
                }
            }

            if( type.compare("float64") == 0 )
            {
                double value = ReadFromBufferAndMoveForward<double>( &buffer_ptr );
                appendToVector( float_index, container->vect_double, field_name, value, canAppend);
            }
            else if( type.compare("float32") == 0 )
            {
                double value = ReadFromBufferAndMoveForward<float>( &buffer_ptr );
                appendToVector( float_index, container->vect_double, field_name, value, canAppend);
            }
            else if( type.compare("uint64") == 0 )
            {
                int64_t value = ReadFromBufferAndMoveForward<uint64_t>( &buffer_ptr );
                appendToVector( int_index, container->vect_integer, field_name, value, canAppend);
            }
            else if( type.compare("int64") == 0 )
            {
                int64_t value = ReadFromBufferAndMoveForward<int64_t>( &buffer_ptr );
                appendToVector( int_index, container->vect_integer, field_name, value, canAppend);
            }
            else if( type.compare("uint32") == 0 )
            {
                int64_t value = ReadFromBufferAndMoveForward<uint32_t>( &buffer_ptr );
                appendToVector( int_index, container->vect_integer, field_name, value, canAppend);
            }
            else if( type.compare("int32") == 0 )
            {
                int64_t value = ReadFromBufferAndMoveForward<int32_t>( &buffer_ptr );
                appendToVector( int_index, container->vect_integer, field_name, value, canAppend);
            }
            else if( type.compare("uint16") == 0 )
            {
                int64_t value = ReadFromBufferAndMoveForward<uint16_t>( &buffer_ptr );
                appendToVector( int_index, container->vect_integer, field_name, value, canAppend);
            }
            else if( type.compare("int16") == 0 )
            {
                int64_t value = ReadFromBufferAndMoveForward<int16_t>( &buffer_ptr );
                appendToVector( int_index, container->vect_integer, field_name, value, canAppend);
            }
            else if( type.compare("uint8") == 0 )
            {
                int64_t value = ReadFromBufferAndMoveForward<uint8_t>( &buffer_ptr );
                appendToVector( int_index, container->vect_integer, field_name, value, canAppend);
            }
            else if( type.compare("int8") == 0 )
            {
                int64_t value = ReadFromBufferAndMoveForward<int8_t>( &buffer_ptr );
                appendToVector( int_index, container->vect_integer, field_name, value, canAppend);
            }
            else if( type.compare("time") == 0 )
            {
                ros::Time value = ReadFromBufferAndMoveForward<ros::Time>( &buffer_ptr );
                appendToVector( time_index, container->vect_time, field_name, value, canAppend);
            }
            else if( type.compare("string") == 0 )
            {
                std::string value;
                int32_t string_size = ReadFromBufferAndMoveForward<int32_t>( &buffer_ptr );

                value.reserve( string_size );
                value.append( (const char*)buffer_ptr, string_size );

                buffer_ptr += string_size;

                appendToVector( string_index, container->vect_string, field_name, value, canAppend);
            }
        }
    }

}

} //namespace RosFlatTypeParser
