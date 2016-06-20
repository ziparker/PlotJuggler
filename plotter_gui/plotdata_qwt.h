#ifndef PLOTDATA_QWT_H
#define PLOTDATA_QWT_H

#include <QColor>
#include <qwt_series_data.h>
#include "plotdata.h"


<<<<<<< HEAD
class PlotDataQwt: public QwtSeriesData<QPointF>
=======
class PlotDataQwt: public PlotData, public QwtSeriesData<QPointF>
>>>>>>> b821eb0b426188e0e1635851b6eb4b29d82b171e
{
public:

   // PlotDataQwt();

<<<<<<< HEAD
    PlotDataQwt(PlotDataPtr base);
=======
    PlotDataQwt(const PlotData& other);
>>>>>>> b821eb0b426188e0e1635851b6eb4b29d82b171e

    virtual ~PlotDataQwt() {}

    virtual QPointF sample( size_t i ) const;
    virtual QRectF boundingRect() const;

    QRectF maximumBoundingRect();

    virtual size_t size() const;

   // void setRangeX(double x_left, double x_right);

    QColor colorHint() const;
    void setColorHint(QColor color);

    void setSubsampleFactor();

private:
    PlotDataPtr _plot_data;
    int _preferedColor;

<<<<<<< HEAD
    int _subsample;
=======
 //   int _subsample;
>>>>>>> b821eb0b426188e0e1635851b6eb4b29d82b171e
  //  int _index_first, _index_last;
};



#endif // PLOTDATA_H
