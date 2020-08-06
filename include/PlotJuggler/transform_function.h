#ifndef TRANSFORM_FUNCTION_H
#define TRANSFORM_FUNCTION_H

#include <set>
#include "PlotJuggler/plotdata.h"
#include "PlotJuggler/pj_plugin.h"

class SeriesTransform : public PlotJugglerPlugin
{
  Q_OBJECT
public:

  SeriesTransform(): _src_data(nullptr) {}

  void setDataSource(PlotData *src_data){
    _src_data = src_data;
  }

  virtual ~SeriesTransform() {}

  virtual void calculate(PlotData* dst_data) = 0;

  virtual void reset() = 0;

  const PlotData* dataSource() const{
    return _src_data;
  }

signals:
  void parametersChanged();

protected:

  PlotData *_src_data;

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
