#ifndef CUSTOMTRACKER_H
#define CUSTOMTRACKER_H


#include <qwt_plot_picker.h>
#include <qwt_picker_machine.h>
#include <qwt_plot_marker.h>
#include <QEvent>

class QwtPlotCurve;

/*
 *
    QwtPlotMarker *mark=new QwtPlotMarker;
    mark->setLinePen(QPen(Qt::red));
    mark->setLineStyle(QwtPlotMarker::VLine);

    QwtText mark_text;

    mark_text.setColor( Qt::black );

    QColor c( "#FFFFFF" );
    mark_text.setBorderPen( QPen( c, 2 ) );
    c.setAlpha( 200 );
    mark_text.setBackgroundBrush( c );

    QString info("default");
    mark_text.setText( info );

    mark->setLabel(mark_text);
    mark->setValue(50,0);//here you have to set the coordinate axis i.e. where the axis are meeting.
    mark->attach(this);
*/

class CurveTracker: public QObject
{
    Q_OBJECT
public:
    explicit CurveTracker(QwtPlot * );

    QPointF actualPosition() const;

    void setEnabled(bool enable);

public slots:
    void manualMove(const QPointF & );
    void refreshPosition( );


private:
    QLineF  curveLineAt( const QwtPlotCurve *, double x ) const;

    QPointF transform( QPoint);
    QPoint  invTransform( QPointF);

    QPointF _prev_trackerpoint;
    std::vector<QwtPlotMarker*> _marker;
    QwtPlotMarker* _line_marker;
    QwtPlotMarker* _text_marker;
    QwtPlot* _plot;
    bool _visible;

signals:
    void timePointMoved(double time);
};

#endif // CUSTOMTRACKER_H
