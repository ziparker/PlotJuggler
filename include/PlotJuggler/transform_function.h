#ifndef TRANSFORM_FUNCTION_H
#define TRANSFORM_FUNCTION_H

#include <set>
#include "PlotJuggler/plotdata.h"
#include "PlotJuggler/pj_plugin.h"

class TimeSeriesTransform : public PlotJugglerPlugin
{
  Q_OBJECT
public:

  TimeSeriesTransform(): _src_data(nullptr) {}

  void setDataSource(const PlotData *src_data){
    _src_data = src_data;
  }

  virtual ~TimeSeriesTransform() {}

  virtual void calculate(PlotData* dst_data) = 0;

  virtual void reset() = 0;

  const PlotData* dataSource() const{
    return _src_data;
  }

signals:
  void parametersChanged();

protected:

  const PlotData *_src_data;
};

QT_BEGIN_NAMESPACE
#define TimeSeriesTransform_iid "com.icarustechnology.PlotJuggler.TimeSeriesTransform"
Q_DECLARE_INTERFACE(TimeSeriesTransform, TimeSeriesTransform_iid)
QT_END_NAMESPACE

///------ The factory to create instances of a SeriesTransform -------------

using TimeSeriesTransformPtr = std::shared_ptr<TimeSeriesTransform>;

class TransformFactory
{
public:

private:
  TransformFactory() {}
  TransformFactory(const TransformFactory&) = delete;
  TransformFactory& operator=(const TransformFactory&) = delete;

  std::unordered_map<std::string, std::function<TimeSeriesTransformPtr()>> creators_;
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

  static TimeSeriesTransformPtr create(const std::string& name)
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
