#pragma once

#include <QApplication>
#include <set>
#include "PlotJuggler/plotdata.h"
#include "PlotJuggler/pj_plugin.h"

class TimeSeriesTransform : public PlotJugglerPlugin
{
  Q_OBJECT
public:

  TimeSeriesTransform(): _src_data(nullptr)
  { }

  void setDataSource(const PlotData *src_data){
    _src_data = src_data;
  }

  virtual ~TimeSeriesTransform() {}

  virtual const char* name() const = 0;

  virtual void calculate(PlotData* dst_data) = 0;

  const PlotData* dataSource() const{
    return _src_data;
  }

  QString alias() const {
    return _alias;
  }

  void setAlias(QString alias) {
    _alias = alias;
  }

signals:
  void parametersChanged();

protected:

  const PlotData *_src_data;
  QString _alias;
};

QT_BEGIN_NAMESPACE
#define TimeSeriesTransform_iid "com.icarustechnology.PlotJuggler.TimeSeriesTransform"
Q_DECLARE_INTERFACE(TimeSeriesTransform, TimeSeriesTransform_iid)
QT_END_NAMESPACE

///------ The factory to create instances of a SeriesTransform -------------

using TimeSeriesTransformPtr = std::shared_ptr<TimeSeriesTransform>;

class TransformFactory: public QObject
{
public:
  TransformFactory() {}

private:
  TransformFactory(const TransformFactory&) = delete;
  TransformFactory& operator=(const TransformFactory&) = delete;

  std::map<std::string, std::function<TimeSeriesTransformPtr()>> creators_;
  std::set<std::string> names_;

  static TransformFactory* instance();

public:

  static const std::set<std::string>& registeredTransforms() {
    return instance()->names_;
  }

  template <typename T> static void registerTransform()
  {
    T temp;
    std::string name = temp.name();
    instance()->names_.insert(name);
    instance()->creators_[name] = [](){ return std::make_shared<T>(); };
  }

  static TimeSeriesTransformPtr create(const std::string& name)
  {
    auto it = instance()->creators_.find(name);
    if( it == instance()->creators_.end())
    {
      return {};
    }
    return it->second();
  }
};

Q_DECLARE_OPAQUE_POINTER(TransformFactory *)
Q_DECLARE_METATYPE(TransformFactory *)
Q_GLOBAL_STATIC(TransformFactory, _transform_factory_ptr_from_macro)

inline TransformFactory* TransformFactory::instance()
{
  static TransformFactory * _ptr(nullptr);
  if (!qApp->property("TransformFactory").isValid() && !_ptr) {
    _ptr = _transform_factory_ptr_from_macro;
    qApp->setProperty("TransformFactory", QVariant::fromValue(_ptr));
  }
  else if (!_ptr) {
    _ptr = qvariant_cast<TransformFactory *>(qApp->property("TransformFactory"));
  }
  else if (!qApp->property("TransformFactory").isValid()) {
    qApp->setProperty("TransformFactory", QVariant::fromValue(_ptr));
  }
  return _ptr;
}

