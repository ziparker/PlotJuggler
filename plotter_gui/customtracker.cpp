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

    _text_marker = ( new QwtPlotMarker );
    _text_marker->attach(plot);

}

QPointF CurveTracker::actualPosition() const
{
    return _prev_trackerpoint;
}

void CurveTracker::setEnabled(bool enable)
{
    _visible = enable;

    _line_marker->setVisible( enable );
    _text_marker->setVisible( enable );

    for (int i=0; i<  _marker.size(); i++)
    {
        _marker[i]->setVisible(enable);
    }
}

void CurveTracker::manualMove(const QPointF& plot_pos)
{
    _prev_trackerpoint = plot_pos;
    refreshPosition();

}

void CurveTracker::refreshPosition()
{
    const QwtPlotItemList curves = _plot->itemList( QwtPlotItem::Rtti_PlotCurve );

    _line_marker->setValue( _prev_trackerpoint );

    double tot_Y = 0;

    for (int i = _marker.size() ; i < curves.size(); i++ )
    {
        _marker.push_back( new QwtPlotMarker );
        _marker[i]->attach( _plot );
        _marker[i]->setVisible( _visible );
    }
    _marker.resize( curves.size() );

    QString text_marker_info;
    double text_X_offset = 0;

    for ( int i = 0; i < curves.size(); i++ )
    {
        QwtPlotCurve *curve = static_cast<QwtPlotCurve *>(curves[i]);
        QColor color = curve->pen().color();


        text_X_offset = 0;// TODO curve->boundingRect().width() * 0.02;

        if( !_marker[i]->symbol() )
        {
            QwtSymbol *sym = new QwtSymbol(
                        QwtSymbol::Diamond,
                        color,
                        color,
                        QSize(5,5));
            _marker[i]->setSymbol(sym);
        }

        const QLineF line = curveLineAt( curve, _prev_trackerpoint.x() );
        QPointF point;
        float middle_X = (line.p1().x() + line.p2().x()) / 2.0;

        if(  _prev_trackerpoint.x() < middle_X )
            point = line.p1();
        else
            point = line.p2();

        tot_Y += point.y();
        _marker[i]->setValue( point );

        text_marker_info += QString( "<font color=""%1"">%2</font>" ).arg( color.name() ).arg( point.y() );

        if(  i < curves.size()-1 ){
            text_marker_info += "<br>";
        }
    }

    QwtText mark_text;
    mark_text.setColor( Qt::black );

    QColor c( "#FFFFFF" );
    mark_text.setBorderPen( QPen( c, 2 ) );
    c.setAlpha( 200 );
    mark_text.setBackgroundBrush( c );

    mark_text.setText( text_marker_info );

    _text_marker->setLabel(mark_text);
    _text_marker->setLabelAlignment( Qt::AlignRight );
    _text_marker->setXValue( _prev_trackerpoint.x() + text_X_offset );
    _text_marker->setYValue( tot_Y/curves.size() );

}


QLineF CurveTracker::curveLineAt(
        const QwtPlotCurve *curve, double x ) const
{
    QLineF line;

    if ( curve->dataSize() >= 2 )
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

    return line;
}

