#include "plotlegend.h"
#include <QEvent>
#include <QMouseEvent>
#include <QWheelEvent>


QRectF PlotLegend::hideButtonRect() const
{
    auto canvas_rect = _parent_plot->canvas()->rect();
    if( alignment() & Qt::AlignRight)
    {
        return QRectF( geometry(canvas_rect).topRight(), QSize(8,-8) );
    }
    return QRectF( geometry(canvas_rect).topLeft(), QSize(-8,-8) );
}

void PlotLegend::draw(QPainter *painter, const QwtScaleMap &xMap,
                      const QwtScaleMap &yMap,
                      const QRectF &rect) const
{
    if( !_collapsed )
    {
        QwtPlotLegendItem::draw(painter, xMap, yMap, rect);
    }

    QRectF iconRect = hideButtonRect();

    if( isVisible() && plotItems().size() > 0)
    {
        painter->save();

        auto col = _collapsed ? Qt::black : Qt::white;
        painter->setPen( col );
        painter->setBrush( QBrush(col, Qt::SolidPattern) );
        painter->drawRect( iconRect );

        QPen black_pen (Qt::black);
        black_pen.setWidth(2);
        painter->setPen( black_pen );
//        painter->drawLine( iconRect.topLeft(), iconRect.bottomRight() );
//        painter->drawLine( iconRect.topRight(), iconRect.bottomLeft() );
        painter->drawEllipse( iconRect );

        painter->restore();
    }
}

void PlotLegend::drawLegendData( QPainter *painter,
    const QwtPlotItem *plotItem, const QwtLegendData &data,
    const QRectF &rect ) const
{
    Q_UNUSED( plotItem );

    const int m = margin();
    const QRectF r = rect.toRect().adjusted( m, m, -m, -m );

    painter->setClipRect( r, Qt::IntersectClip );

    int titleOff = 0;

    const QwtGraphic graphic = data.icon();
    if ( !graphic.isEmpty() )
    {
        QRectF iconRect( r.topLeft(), graphic.defaultSize() );

        iconRect.moveCenter(
            QPoint( iconRect.center().x(), rect.center().y() ) );

        if(  plotItem->isVisible() )
        {
            graphic.render( painter, iconRect, Qt::KeepAspectRatio );
        }

        titleOff += iconRect.width() + spacing();
    }

    const QwtText text = data.title();
    if ( !text.isEmpty() )
    {
        auto pen = textPen();
        if( !plotItem->isVisible() )
        {
            pen.setColor( Qt::gray );
        }
        painter->setPen( pen );
        painter->setFont( font() );

        const QRectF textRect = r.adjusted( titleOff, 0, 0, 0 );
        text.draw( painter, textRect );
    }
}


bool PlotLegend::processWheelEvent(QWheelEvent* mouse_event)
{
    if ( mouse_event->modifiers() == Qt::ControlModifier && isVisible() )
    {
        auto canvas_rect = _parent_plot->canvas()->rect();
        auto legend_rect = geometry( canvas_rect );
        if( legend_rect.contains( mouse_event->pos()) )
        {
            int point_size = font().pointSize();
            if( mouse_event->delta() > 0 && point_size < 14)
            {
                emit legendSizeChanged(point_size+1);
            }
            if( mouse_event->delta() < 0 && point_size > 6)
            {
                emit legendSizeChanged(point_size-1);
            }
            return true;
        }
    }
    return false;
}

const QwtPlotItem* PlotLegend::processMousePressEvent(QMouseEvent* mouse_event)
{
    auto canvas_rect = _parent_plot->canvas()->rect();
    const QPoint press_point = mouse_event->pos();

    if( isVisible() &&  mouse_event->modifiers() == Qt::NoModifier)
    {
        if( !_collapsed && geometry( canvas_rect ).contains(press_point) )
        {
            for(auto item: plotItems())
            {
                auto item_rect = legendGeometries(item).first();
                if( item_rect.contains( press_point ))
                {
                    return item;
                }
            }
        }
        else if( hideButtonRect().contains(press_point) )
        {
            _collapsed = !_collapsed;
            _parent_plot->replot();
            return nullptr;
        }
    }
    return nullptr;
}
