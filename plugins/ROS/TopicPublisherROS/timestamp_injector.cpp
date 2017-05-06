#include "timestamp_injector.h"

using namespace RosIntrospection;

template <typename T> T readFromBuffer( uint8_t** buffer)
{
  T destination =  (*( reinterpret_cast<T*>( *buffer ) ) );
  *buffer +=  sizeof(T);
  return destination;
}

inline void SkipBytesInBuffer( uint8_t** buffer, int vector_size, const BuiltinType& type )
{
    if( type == STRING)
    {
        for (int i=0; i<vector_size; i++){
            int32_t string_size = readFromBuffer<int32_t>( buffer );
            *buffer += string_size;
        }
    }
    else{
        *buffer += vector_size * BuiltinTypeSize[ static_cast<int>(type) ];
    }
}
//------------------------------------------------
void injectTimeImpl(const ROSTypeList& type_list,
                    const ROSType &type,
                    bool header_timestamp_message,
                    uint8_t** buffer_ptr,
                    const ros::Time& new_timestamp)
{
    int array_size = type.arraySize();
    if( array_size == -1)
    {
        array_size = readFromBuffer<int32_t>( buffer_ptr );
    }

    if( type.typeID() == OTHER ) // recursion
    {
        for (int v=0; v<array_size; v++)
        {
            for(const ROSMessage& msg: type_list) // find in the list
            {
                if( msg.type().msgName() == type.msgName() &&
                    msg.type().pkgName() == type.pkgName()  )
                {
                    for (const ROSField& field : msg.fields() )
                    {
                        if( !field.isConstant()) {

                            bool is_header_timestamp = (
                                    field.type().typeID()  == TIME &&
                                    field.type().isArray() == false &&
                                    field.name() == "stamp" &&
                                    msg.type().msgName() == "Header" &&
                                    msg.type().pkgName() == "std_msgs" );

                            if( !is_header_timestamp &&
                                    field.type().typeID() != OTHER &&
                                    field.type().typeID() != STRING
                                    && field.type().arraySize() == 1)
                            {
                                //quick skip
                                *buffer_ptr += BuiltinTypeSize[ static_cast<int>(field.type().typeID()) ];
                            }
                            else{
                                //recursion
                                injectTimeImpl(type_list, field.type(), is_header_timestamp, buffer_ptr, new_timestamp);
                            }
                        }
                    }
                    break;
                }
            }
        }
    }
    else if( header_timestamp_message )
    {
        uint32_t *sec = reinterpret_cast<uint32_t*>( *buffer_ptr );
        *buffer_ptr += sizeof(uint32_t);
        uint32_t *nsec = reinterpret_cast<uint32_t*>( *buffer_ptr );
        *buffer_ptr += sizeof(uint32_t);
        *sec  = new_timestamp.sec;
        *nsec = new_timestamp.nsec;
    }
    else{
        SkipBytesInBuffer( buffer_ptr, array_size, type.typeID() );
    }
}





void injectTime(const ROSTypeList &type_map, ROSType type, uint8_t *buffer_ptr, const ros::Time &new_timestamp)
{
    uint8_t** buffer = &buffer_ptr;
    injectTimeImpl( type_map, type, false, buffer, new_timestamp );
}
