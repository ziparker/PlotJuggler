#ifndef PLOTDATA_QWT_H
#define PLOTDATA_QWT_H

#include <QColor>
#include <qwt_series_data.h>
#include "PlotJuggler/plotdata.h"



class PlotDataQwt: public QwtSeriesData<QPointF>
{
public:

    PlotDataQwt(PlotDataPtr base);

    virtual ~PlotDataQwt() {}

    virtual QPointF sample( size_t i ) const override;

    virtual QRectF boundingRect() const override;

    virtual size_t size() const override;

    QRectF maximumBoundingRect(double min_X, double max_X);

    PlotDataPtr data() { return _plot_data; }

    QColor randomColorHint() const;
    void setColorHint(QColor color);

    void setSubsampleFactor();

private:
    PlotDataPtr _plot_data;
    int      _preferedColor;
    unsigned _subsample;
};



#endif // PLOTDATA_H
