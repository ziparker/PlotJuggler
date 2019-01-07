#pragma once

#include <memory>
#include <string>
#include <QWidget>
#include <QString>
#include <QDomDocument>
#include <QJSEngine>
#include "PlotJuggler/plotdata.h"

class CustomFunction;
class QJSEngine;
typedef std::shared_ptr<CustomFunction> CustomPlotPtr;

struct SnippetData{
    QString name;
    QString globalVars;
    QString equation;
};

class CustomFunction
{
public:
    CustomFunction(const std::string &linkedPlot,
               const std::string &plotName,
               const QString &globalVars,
               const QString &function);

    void calculateAndAdd(PlotDataMapRef &plotData);

    void calculate(const PlotDataMapRef &plotData, PlotData *dst_data);

    const std::string& name() const;

    const std::string& linkedPlotName() const;

    const QString& globalVars() const;

    const QString& function() const;

    QDomElement xmlSaveState(QDomDocument &doc) const;

    static CustomPlotPtr createFromXML(QDomElement &element );

private:
    void initJsEngine();

    PlotData::Point  calculatePoint(QJSValue &calcFct,
                                    const PlotData &src_data,
                                    const std::vector<const PlotData *> &channels_data,
                                    QJSValue &chan_values,
                                    size_t point_index);

    std::string _linked_plot_name;
    std::string _plot_name;
    QString _global_vars;
    QString _function;
    std::vector<std::string> _used_channels;

    std::shared_ptr<QJSEngine> _jsEngine;
    double _last_updated_timestamp;
};

