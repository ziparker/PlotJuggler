#include "custom_transform.h"

#include <limits>
#include <QFile>
#include <QMessageBox>

CustomTransform::CustomTransform(const QString &globalVars,
                                 const QString &function):
    _global_vars(globalVars),
    _function(function),
    _last_updated_timestamp( - std::numeric_limits<double>::max() )
{
    initJsEngine();
}

void CustomTransform::initJsEngine()
{
    _jsEngine = std::make_shared<QJSEngine>();

    QJSValue globalVarResult = _jsEngine->evaluate(_global_vars);
    if(globalVarResult.isError())
    {
        throw std::runtime_error("JS Engine : " + globalVarResult.toString().toStdString());
    }
    QString calcMethodStr = QString("function calc(time, value){with (Math){\n%1\n}}").arg(_function);
    _jsEngine->evaluate(calcMethodStr);

    static QStringList files{":/js/resources/common.js"};

    for(QString fileName : files)
    {
        QFile file(fileName);
        if(file.open(QIODevice::ReadOnly))
        {
            QString commonData = QString::fromUtf8(file.readAll());
            QJSValue out = _jsEngine->evaluate(commonData);
            if(out.isError())
            {
                qWarning() << "JS Engine : " << out.toString();
            }
        }
    }
}

QPointF CustomTransform::calculatePoint(QJSValue& calcFct,
                                        const PlotData& src_data,
                                        size_t point_index)
{
    const PlotData::Point &old_point = src_data.at(point_index);

    QPointF new_point;
    new_point.setX( old_point.x );

    QJSValue jsData = calcFct.call({QJSValue(old_point.x), QJSValue(old_point.y)});
    if(jsData.isError())
    {
        throw std::runtime_error("JS Engine : " + jsData.toString().toStdString());
    }
    new_point.setY( jsData.toNumber() );
    return new_point;
}

void CustomTransform::calculate(const PlotData& src_data, std::deque<QPointF>* dst_data)
{
    QJSValue calcFct = _jsEngine->evaluate("calc");

    if(calcFct.isError())
    {
        throw std::runtime_error("JS Engine : " + calcFct.toString().toStdString());
    }

    // clean up old ones
    const double first_time = src_data.front().x;
    while(dst_data->size() > 0 && dst_data->front().x() <  first_time)
    {
        dst_data->pop_front();
    }

    for(size_t i=0; i < src_data.size(); ++i)
    {
        if( src_data.at(i).x > _last_updated_timestamp)
        {
            dst_data->push_back( calculatePoint(calcFct, src_data, i ) );
        }
    }
    _last_updated_timestamp = dst_data->back().x();
}

const QString &CustomTransform::globalVars() const
{
    return _global_vars;
}

const QString &CustomTransform::function() const
{
    return _function;
}

CustomTransformPtr CustomTransform::createFromXML(QDomElement &element)
{
    auto globalVars = element.firstChildElement("global").text().trimmed();
    auto calcEquation = element.firstChildElement("equation").text().trimmed();
    return std::make_shared<CustomTransform>( globalVars, calcEquation );
}

