#ifndef PLOTDATA_QWT_H
#define PLOTDATA_QWT_H



#include <QColor>
#include <qwt_series_data.h>
#include "plotdata.h"



class PlotDataQwt: public PlotData, public QwtSeriesData<QPointF>
{
public:

    PlotDataQwt(SharedVector x, SharedVector y, std::string name);

    virtual ~PlotDataQwt() {}

    virtual QPointF sample( size_t i ) const;
    virtual QRectF boundingRect() const;

    virtual size_t size() const;


    QColor colorHint() const;
    void setColorHint(QColor color);

private:

    int _preferedColor;
    QColor _color;
    QString _name;
};

typedef std::shared_ptr<PlotDataQwt> PlotDataQwtPtr;
typedef std::map<QString, PlotDataQwtPtr> PlotDataQwtMap;


#endif // PLOTDATA_H
