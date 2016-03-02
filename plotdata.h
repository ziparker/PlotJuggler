#ifndef PLOTDATA_H
#define PLOTDATA_H

#include <QObject>
#include <qwt_series_data.h>

class PlotData: public QwtSeriesData<QPointF>
{
public:
    PlotData();

    virtual size_t size() const;
    virtual QPointF sample( size_t i ) const;

    virtual QRectF boundingRect() const;

};

#endif // PLOTDATA_H
