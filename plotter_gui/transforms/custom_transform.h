#pragma once

#include <memory>
#include <string>
#include <QWidget>
#include <QString>
#include <QDomDocument>
#include <QJSEngine>
#include "PlotJuggler/plotdata.h"

class CustomTransform;
class QJSEngine;
typedef std::shared_ptr<CustomTransform> CustomTransformPtr;

class CustomTransform
{
public:
    CustomTransform(const QString &global_vars,
                    const QString &function);

    void calculate(const PlotData& src_data, std::deque<QPointF> *destination);

    const QString& globalVars() const;

    const QString& function() const;

    static CustomTransformPtr createFromXML(QDomElement &element );

private:
    void initJsEngine();

    QPointF calculatePoint(QJSValue &calcFct,
                                    const PlotData &src_data,
                                    size_t point_index);
    QString _global_vars;
    QString _function;

    std::shared_ptr<QJSEngine> _jsEngine;
    double _last_updated_timestamp;
};

