#ifndef TRANSFORM_FUNCTION_H
#define TRANSFORM_FUNCTION_H

#include <set>
#include "PlotJuggler/plotdata.h"
#include "PlotJuggler/pj_plugin.h"

class SeriesTransform : public PlotJugglerPlugin
{
public:

  SeriesTransform() {}

  virtual ~SeriesTransform() {}

  void calculate(const PlotData& src_data, PlotData* dst_data);

  virtual void reset() = 0;

protected:

  virtual PlotData::Point calculatePoint(const PlotData& src_data, size_t point_index) = 0;

};

QT_BEGIN_NAMESPACE
#define SeriesTransform_iid "com.icarustechnology.PlotJuggler.SeriesTransform"
Q_DECLARE_INTERFACE(SeriesTransform, SeriesTransform_iid)
QT_END_NAMESPACE

///------ The factory to create instances of a SeriesTransform -------------

using SeriesTransformPtr = std::shared_ptr<SeriesTransform>;

class TransformFactory
{
public:

private:
  TransformFactory() {}
  TransformFactory(const TransformFactory&) = delete;
  TransformFactory& operator=(const TransformFactory&) = delete;

  std::unordered_map<std::string, std::function<SeriesTransformPtr()>> creators_;
  std::set<std::string> names_;

  static TransformFactory& get()
  {
    static TransformFactory instance_;
    return instance_;
  }

public:

  static const std::set<std::string>& registeredNames()
  {
    return get().names_;
  }

  template <typename T> static void registerTransform(const std::string& name )
  {
    get().names_.insert(name);
    get().creators_[name] = [](){ return std::make_shared<T>(); };
  }

  static SeriesTransformPtr create(const std::string& name)
  {
    auto it = get().creators_.find(name);
    if( it == get().creators_.end())
    {
      return {};
    }
    return it->second();
  }

};


#endif // TRANSFORM_FUNCTION_H
