#ifndef SHAPE_SHIFTER_FACTORY_HPP
#define SHAPE_SHIFTER_FACTORY_HPP

#include <ros_type_introspection/shape_shifter_2.hpp>

class ShapeShifterFactory{
public:
  static ShapeShifterFactory &getInstance();

  void registerMessage(const std::string& topic_name, const std::__cxx11::string &md5sum, const std::string& datatype, const std::string& definition );

  boost::optional<RosIntrospection::ShapeShifter2 *> getMessage(const std::string& topic_name);

  const std::vector<std::string>& getTopicList() const;

private:
  ShapeShifterFactory() = default;
  std::map<std::string, RosIntrospection::ShapeShifter2> map_;
  std::vector<std::string> topics_;

};

#endif // SHAPE_SHIFTER_FACTORY_HPP

inline ShapeShifterFactory& ShapeShifterFactory::getInstance()
{
  static ShapeShifterFactory instance;
  return instance;
}

inline void ShapeShifterFactory::registerMessage(const std::string &topic_name,
                                                 const std::string &md5sum,
                                                 const std::string &datatype,
                                                 const std::string &definition)
{
  RosIntrospection::ShapeShifter2 msg;
  msg.morph(md5sum, datatype,definition);

  if( map_.count(topic_name) == 0)
  {
    map_.insert( std::make_pair(topic_name, std::move(msg) ));
    topics_.push_back( topic_name );
  }

}

inline boost::optional<RosIntrospection::ShapeShifter2*> ShapeShifterFactory::getMessage(const std::string &topic_name)
{
  auto it = map_.find( topic_name );
  if( it == map_.end())
    return boost::optional<RosIntrospection::ShapeShifter2*>();
  else
    return boost::optional<RosIntrospection::ShapeShifter2*>( &(it->second) );
}

inline const std::vector<std::string> &ShapeShifterFactory::getTopicList() const
{
  return topics_;
}


