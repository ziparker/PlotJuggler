#ifndef QML_CUSTOM_FUNCTION_H
#define QML_CUSTOM_FUNCTION_H

#include "custom_function.h"
#include <QJSEngine>

class QmlCustomFunction: public CustomFunction
{
public:
  QmlCustomFunction(const std::string &linkedPlot,
                    const SnippetData &snippet);

  void initEngine() override;

  virtual PlotData::Point calculatePoint(const PlotData & src_data,
                                         const std::vector<const PlotData *> & channels_data,
                                         std::vector<double> & chan_values,
                                         size_t point_index) override;
private:

  std::unique_ptr<QJSEngine> _qml_engine;
  QJSValue _qml_function;
  QJSValue _chan_values_qml;
};
#endif // QML_CUSTOM_FUNCTION_H
