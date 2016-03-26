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
};

class CurveTracker: public QwtPlotPicker
{
    Q_OBJECT
public:
    explicit CurveTracker( QWidget * );

public slots:
    void mirroredMove(const QPointF & );

protected:
    virtual QwtText trackerText( const QPoint &pos ) const;
    virtual QwtText trackerTextF( const QPointF & ) const;
    virtual void move( const QPoint & );
    virtual QRect trackerRect( const QFont &font ) const;
private:
    QString curveInfoAt( const QwtPlotCurve *, const QPointF & ) const;
    QLineF  curveLineAt( const QwtPlotCurve *, double x ) const;

    QPointF _prev_trackerpoint;
};

#endif // CUSTOMTRACKER_H
