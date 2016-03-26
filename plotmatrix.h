#ifndef _PLOT_MATRIX_H_
#define _PLOT_MATRIX_H_

#include <qframe.h>
#include <qwt_plot.h>
#include <QGridLayout>
#include "plotwidget.h"

class PlotMatrix: public QFrame
{
    Q_OBJECT

public:
    PlotMatrix(QWidget * parent = NULL );
    virtual ~PlotMatrix();

    void addRow();
    void addColumn();
    void removeColumn(int column_to_delete);
    void removeRow(int row_to_delete);

    void removeEmpty();

    int numRows() const;
    int numColumns() const;

    bool isRowEmpty(int row );
    bool isColumnEmpty(int row );

    PlotWidget* plotAt( int row, int column );
    const PlotWidget* plotAt( int row, int column ) const;

    void setAxisScale(int axisId, int row, int col,
        double min, double max, double step = 0 );

    QDomElement getDomElement(QDomDocument &doc);

    void updateLayout();
    void replot();

    void setHorizontalLink(bool linked);
    void setActiveTracker(bool active);

public slots:
    void maximizeHorizontalScale();
    void maximizeVerticalScale();

private slots:
    void swapWidgetByName(QString name_a, QString name_b);
    void onHorizontalAxisScaleChanged(QRectF range);

private:
    void alignAxes( int rowOrColumn, int axis );
    void alignScaleBorder( int rowOrColumn, int axis );
    PlotWidget *addPlotWidget( int row, int col);
    void swapPlots( int rowA, int colA, int rowB, int colB);

    QGridLayout *layout;
    int num_rows;
    int num_cols;
    int widget_uid;
    bool _horizontal_link;

signals:
    void plotAdded(PlotWidget*);
};

#endif
