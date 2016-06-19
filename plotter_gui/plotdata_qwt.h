#ifndef PLOTDATA_QWT_H
#define PLOTDATA_QWT_H

#include <QColor>
#include <qwt_series_data.h>
#include "plotdata.h"


class PlotDataQwt: public PlotData, public QwtSeriesData<QPointF>
{
public:

   // PlotDataQwt();

    PlotDataQwt(const PlotData& other);

    virtual ~PlotDataQwt() {}

    virtual QPointF sample( size_t i ) const;
    virtual QRectF boundingRect() const;

    QRectF maximumBoundingRect();

    virtual size_t size() const;

    void setRangeX(double x_left, double x_right);

    QColor colorHint() const;
    void setColorHint(QColor color);

    void setSubsampleFactor(int factor);

private:

    int _preferedColor;

 //   int _subsample;
  //  int _index_first, _index_last;
};



#endif // PLOTDATA_H
