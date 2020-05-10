#include "ros2_parser.h"
#include "jointstates_msg.h"
#ifdef FOUND_PJ_MSGS
  #include "plotjuggler_msgs.h"
#endif

namespace Ros2Introspection {


void MessageParserBase::setUseHeaderStamp(bool use)
{
  _use_header_stamp = use;
}

PlotData &MessageParserBase::getSeries(PlotDataMapRef &plot_data, const std::string key)
{
  auto plot_pair = plot_data.numeric.find( key );
  if( plot_pair == plot_data.numeric.end() )
  {
    plot_pair = plot_data.addNumeric( key );
  }
  return plot_pair->second;
}

//-------------------------------------
void IntrospectionParser::setMaxArrayPolicy(MaxArrayPolicy discard_policy, size_t max_size)
{
  _intropection_parser.setMaxArrayPolicy(discard_policy, max_size);
}



bool IntrospectionParser::parseMessage(const std::string& topic_name,
                                       PlotDataMapRef& plot_data,
                                       const rcutils_uint8_array_t *serialized_msg,
                                       double timestamp)
{
  _intropection_parser.deserializeIntoFlatMessage(serialized_msg, &_flat_msg);

  if(_use_header_stamp && _intropection_parser.topicInfo().has_header_stamp)
  {
    double sec  = _flat_msg.values[0].second;
    double nsec = _flat_msg.values[1].second;
    timestamp = sec + (nsec*1e-9);
  }

  ConvertFlatMessageToRenamedValues(_flat_msg, _renamed);

  for(const auto& it: _renamed)
  {
      const auto& key = it.first;
      double value = it.second;

      auto plot_pair = plot_data.numeric.find( key );
      if( plot_pair == plot_data.numeric.end() )
      {
          plot_pair = plot_data.addNumeric( key );
      }
      auto& series = plot_pair->second;

      if( !std::isnan(value) && !std::isinf(value) ){
        series.pushBack( {timestamp, value} );
      }
  }
  return true;
}

//-----------------------------------------

CompositeParser::CompositeParser():
  _discard_policy( MaxArrayPolicy::DISCARD_LARGE_ARRAYS ),
  _max_array_size(999)
{

}

void CompositeParser::setUseHeaderStamp(bool use)
{
  _use_header_stamp = use;
  for( auto it: _parsers )
  {
    it.second->setUseHeaderStamp(use);
  }
}

void CompositeParser::setMaxArrayPolicy(MaxArrayPolicy discard_policy, size_t max_size)
{
  _discard_policy = discard_policy;
  _max_array_size = max_size;
  for( auto it: _parsers )
  {
    it.second->setMaxArrayPolicy(discard_policy, max_size);
  }
}

void CompositeParser::registerMessageType(const std::string &topic_name,
                                          const std::string &topic_type)
{
  std::shared_ptr<MessageParserBase> parser;
  if(_parsers.count(topic_name) > 0)
  {
    return;
  }

  std::string type = topic_type;

  // replace verbose name
  size_t str_index = type.find("/msg/", 0);
  if (str_index != std::string::npos)
  {
    type.erase(str_index, 4);
  }

  if( type == "sensor_msgs/JointState"){
    parser.reset( new JointStateMsgParser );
  }
#ifdef FOUND_PJ_MSGS
  else if( type == "pj_msgs/Dictionary"){
    parser.reset( new PlotJugglerDictionaryParser );
  }
  else if( type == "pj_msgs/DataPoints"){
    parser.reset( new PlotJugglerDataPointsParser );
  }
#endif
  else {
    parser.reset( new IntrospectionParser(topic_name, type) );
  }

  parser->setMaxArrayPolicy(_discard_policy, _max_array_size);
  parser->setUseHeaderStamp(_use_header_stamp);
  _parsers.insert( { topic_name, parser} );
}

bool CompositeParser::parseMessage(const std::string &topic_name,
                                   PlotDataMapRef &plot_data,
                                   const rcutils_uint8_array_t *serialized_msg,
                                   double timestamp)
{
  auto it = _parsers.find(topic_name);
  if( it == _parsers.end() )
  {
    return false;
  }
  it->second->parseMessage(topic_name, plot_data, serialized_msg, timestamp);
  return false;
}

const rosidl_message_type_support_t *CompositeParser::typeSupport(const std::string &topic_name) const
{
  auto it = _parsers.find(topic_name);
  if( it == _parsers.end() )
  {
    return nullptr;
  }
  return it->second->typeSupport();
}

}
