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

    QPointF front() { return _raw_points.front(); }
    QPointF back()  { return _raw_points.back(); }

    void setSubsampleFactor(int factor);
    virtual size_t size() const;
    virtual QPointF sample( size_t i ) const;

    virtual QRectF boundingRect() const;

    void setRangeX(float t_left, float t_right);
private:
    std::vector<QPointF> _raw_points;
    int _subsample;

    float y_min, y_max;
    float x_min, x_max;
    int _indexA, _indexB;

    int _preferedColor;
};

#endif // PLOTDATA_H
