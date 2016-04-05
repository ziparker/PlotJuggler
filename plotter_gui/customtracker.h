#ifndef CUSTOMTRACKER_H
#define CUSTOMTRACKER_H


#include <qwt_plot_picker.h>
#include <qwt_picker_machine.h>
#include <QEvent>

class QwtPlotCurve;

class QWT_EXPORT PickerTrackerMachine: public QwtPickerMachine
{
public:
    PickerTrackerMachine();

    virtual QList<Command> transition(const QwtEventPattern &, const QEvent * );
private:
    bool keypressed;
};

class CurveTracker: public QwtPlotPicker
{
    Q_OBJECT
public:
    explicit CurveTracker( QWidget * );

    QPointF actualPosition() const;

public slots:
    void manualMove(const QPointF & );
    void onExternalZoom( const QRectF &);

protected:
    virtual QwtText trackerText( const QPoint &pos ) const;
    virtual QwtText trackerTextF(QPointF ) const;
    virtual void move( const QPoint & );
    virtual QRect trackerRect( const QFont &font ) const;
private:
    QString curveInfoAt(const QwtPlotCurve *, QPointF ) const;
    QLineF  curveLineAt( const QwtPlotCurve *, double x ) const;

    QPointF _prev_trackerpoint;
};

#endif // CUSTOMTRACKER_H
