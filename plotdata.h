#ifndef PLOTDATA_H
#define PLOTDATA_H

#include <vector>
#include <QObject>
#include <qwt_series_data.h>

class PlotData: public QwtSeriesData<QPointF>
{
public:
    PlotData(int capacity);

    void reserve(size_t capacity);
    void pushBack(QPointF point);
    void setSubsampleFactor(int factor);
    virtual size_t size() const;
    virtual QPointF sample( size_t i ) const;

    virtual QRectF boundingRect() const;
private:
    std::vector<QPointF> _raw_points;
    int _subsample;

    float y_min, y_max;

    int _preferedColor;
};

#endif // PLOTDATA_H
