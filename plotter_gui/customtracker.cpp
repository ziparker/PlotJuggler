#include "customtracker.h"
#include <qwt_series_data.h>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include "qwt_event_pattern.h"
#include <qwt_symbol.h>
#include <qevent.h>

struct compareX
{
    inline bool operator()( const double x, const QPointF &pos ) const
    {
        return ( x < pos.x() );
    }
};



CurveTracker::CurveTracker( QwtPlot *plot ):
    QObject( plot ),
    _plot( plot )
{
    _line_marker = ( new QwtPlotMarker );

    _line_marker->setLinePen(QPen(Qt::red));
    _line_marker->setLineStyle(QwtPlotMarker::VLine);
    _line_marker->setValue(0,0);
    _line_marker->attach(plot);

}

QPointF CurveTracker::actualPosition() const
{
    return _prev_trackerpoint;
}

void CurveTracker::setEnabled(bool enable)
{
    _visible = enable;
    _line_marker->setVisible( enable );

    for (int i=0; i<  _marker.size(); i++)
    {
        _marker[i]->setVisible(enable);
    }
}

void CurveTracker::manualMove(const QPointF& plot_pos)
{
    _prev_trackerpoint = plot_pos;

    const QwtPlotItemList curves = _plot->itemList( QwtPlotItem::Rtti_PlotCurve );

    _line_marker->setValue(plot_pos );

    for (int i = _marker.size() ; i < curves.size(); i++ )
    {
        _marker.push_back( new QwtPlotMarker );
        _marker[i]->attach( _plot );
        _marker[i]->setVisible( _visible );
    }
    _marker.resize( curves.size() );

    for ( int i = 0; i < curves.size(); i++ )
    {
        QwtPlotCurve *curve = static_cast<QwtPlotCurve *>(curves[i]);
        QColor color = curve->pen().color();

        if( !_marker[i]->symbol() )
        {
            QwtSymbol *sym = new QwtSymbol(
                        QwtSymbol::Diamond,
                        color,
                        color,
                        QSize(5,5));
            _marker[i]->setSymbol(sym);
        }

        const QLineF line = curveLineAt( curve, plot_pos.x() );
        QPointF p1 = line.p1();

        QwtText mark_text;
        mark_text.setColor( color );

        QColor c( "#FFFFFF" );
        mark_text.setBorderPen( QPen( c, 2 ) );
        c.setAlpha( 200 );
        mark_text.setBackgroundBrush( c );

        QString info = QString::number( p1.y() );

        mark_text.setText( info );

        _marker[i]->setLabel(mark_text);
        _marker[i]->setLabelAlignment( Qt::AlignRight );
        _marker[i]->setValue( p1 );
    }
}

void CurveTracker::refreshPosition()
{

}


QLineF CurveTracker::curveLineAt(
        const QwtPlotCurve *curve, double x ) const
{
    QLineF line;

    if ( curve->dataSize() >= 2 )
    {
        const QRectF br = curve->boundingRect();
        if ( ( br.width() > 0 ) && ( x >= br.left() ) && ( x <= br.right() ) )
        {
            int index = qwtUpperSampleIndex<QPointF>(
                        *curve->data(), x, compareX() );

            if ( index == -1 &&
                 x == curve->sample( curve->dataSize() - 1 ).x() )
            {
                // the last sample is excluded from qwtUpperSampleIndex
                index = curve->dataSize() - 1;
            }

            if ( index > 0 )
            {
                line.setP1( curve->sample( index - 1 ) );
                line.setP2( curve->sample( index ) );
            }
        }
    }

    return line;
}

