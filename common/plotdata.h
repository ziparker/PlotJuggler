#ifndef PLOTDATA_H
#define PLOTDATA_H

#include <vector>
#include <QObject>
#include <QColor>
#include <qwt_series_data.h>
#include <QSharedPointer>


typedef QSharedPointer<std::vector<double> > SharedVector;

class PlotData: public QwtSeriesData<QPointF>
{
public:
    PlotData();
    PlotData(SharedVector x, SharedVector y);
    void addData( SharedVector x, SharedVector y );

    void setName(QString name) { _name = name; }
    QString name() { return _name; }

    void setSubsampleFactor(int factor);

    virtual size_t size() const;
    virtual QPointF sample( size_t i ) const;
    virtual QRectF boundingRect() const;

    QColor colorHint() const;
    void setColorHint(QColor color);

    void setRangeX(double t_left, double t_right);

    int getIndexFromX(double x) const;

    double getY(double x ) const;

private:

    SharedVector _x_points;
    SharedVector _y_points;

    int _subsample;

    float y_min, y_max;
    float x_min, x_max;
    int _index_first, _index_last;

    int _preferedColor;
    QColor _color;
    QString _name;
};

#endif // PLOTDATA_H
