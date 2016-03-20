#include <qlayout.h>
#include <qpen.h>
#include <qwt_plot.h>
#include <qwt_plot_canvas.h>
#include <qwt_scale_widget.h>
#include <qwt_scale_draw.h>
#include "plotmatrix.h"




PlotMatrix::PlotMatrix(QWidget *parent ):
    QFrame( parent )
{
    num_rows = 0;
    num_cols = 0;
    widget_uid = 0;

    layout = new QGridLayout( this );

    _horizontal_link = true;
    updateLayout();
}

PlotWidget* PlotMatrix::addPlot(int row, int col)
{
    PlotWidget *plot = new PlotWidget( this );

    plot->setWindowTitle(QString("PlotWidget ") + QString::number(widget_uid++));

    layout->addWidget( plot, row, col );

    qDebug() << "adding "<< row << " " << col;

    connect( plot, SIGNAL(swapWidgets(QString,QString)), this, SLOT(swapWidgetByName(QString,QString)) );
    connect( plot, SIGNAL(horizontalScaleChanged(QRectF)), this, SLOT(onHorizontalAxisScaleChanged(QRectF)));

    plot->setAttribute(Qt::WA_DeleteOnClose);

    emit plotAdded(plot);
    return plot;
}

void PlotMatrix::addRow()
{

    if( num_rows==0 && num_cols==0 )
    {
        addPlot( 0, 0 );
        num_rows = 1;
        num_cols = 1;
    }
    else{
        for ( int col = 0; col < numColumns(); col++ )
        {
            addPlot( num_rows, col );
        }
        num_rows++;
    }
    updateLayout();
}

void PlotMatrix::addColumn()
{
    if( num_rows==0 && num_cols==0 )
    {
        addPlot( 0, 0 );
        num_rows = 1;
        num_cols = 1;
    }
    else {
        for ( int row = 0; row < numRows(); row++ )
        {
            addPlot( row, num_cols );
        }
        num_cols++;
    }
    updateLayout();
}

void PlotMatrix::swapPlots( int rowA, int colA, int rowB, int colB)
{
    QWidget *widgetA = layout->itemAtPosition(rowA, colA)->widget();
    QWidget *widgetB = layout->itemAtPosition(rowB, colB)->widget();

    layout->removeItem( layout->itemAtPosition(rowA, colA) );
    layout->removeItem( layout->itemAtPosition(rowB, colB) );

    layout->addWidget(widgetA, rowB, colB);
    layout->addWidget(widgetB, rowA, colA);
}

void PlotMatrix::removeColumn(int column_to_delete)
{
        qDebug() << " removeColumn " << column_to_delete+1 << " / " << numColumns();

    for(int col = column_to_delete; col< num_cols-1; col++)
    {
        for(int row=0; row< num_rows; row++)
        {
            this->swapPlots( row, col, row, col+1);
        }
    }
    for(int row=0; row< num_rows; row++)
    {
        plotAt( row, num_cols -1)->close();
    }
    num_cols--;
    updateLayout();
}

void PlotMatrix::removeRow(int row_to_delete)
{
    qDebug() << " removeRow " << row_to_delete+1 << " / " << numRows();
    for(int row = row_to_delete; row< num_rows-1; row++)
    {
       for(int col = 0; col< num_cols; col++)
        {
            this->swapPlots( row, col, row+1, col);
        }
    }
    for(int col=0; col< num_cols; col++)
    {
        plotAt( num_rows-1, col)->close();
    }
    num_rows--;
    updateLayout();
}



PlotMatrix::~PlotMatrix(){}

int PlotMatrix::numRows() const
{
    return num_rows;
}

int PlotMatrix::numColumns() const
{
    return num_cols;
}

bool PlotMatrix::isColumnEmpty( int col )
{
    for (int r=0; r< layout->rowCount(); r++)
    {
        QLayoutItem *item = layout->itemAtPosition(r, col);
        if( item  ){
            PlotWidget *widget = static_cast<PlotWidget *>( item->widget() );
            if (widget && widget->isEmpty() == false)
            {
                return false;
            }
        }
    }
    return true;
}

bool PlotMatrix::isRowEmpty(int row )
{
    for (int c=0; c< layout->columnCount(); c++)
    {
        QLayoutItem *item = layout->itemAtPosition(row, c);
        if( item  ){
            PlotWidget *widget = static_cast<PlotWidget *>( item->widget() );
            if (widget && widget->isEmpty() == false)
            {
                return false;
            }
        }
    }
    return true;
}


PlotWidget* PlotMatrix::plotAt( int row, int column )
{
    QLayoutItem* item = layout->itemAtPosition(row,column);
    if(item)
    {
        PlotWidget* plot = static_cast<PlotWidget*>( item->widget() );
        return plot;
    }

    return NULL;
}

const PlotWidget *PlotMatrix::plotAt( int row, int column ) const
{
    QLayoutItem* item = layout->itemAtPosition(row,column);
    if(item)
    {
        PlotWidget* plot = static_cast<PlotWidget*>( item->widget() );
        return plot;
    }
    return NULL;
}



void PlotMatrix::setAxisScale( int axis, int row, int col,
                               double min, double max, double step )
{
    QwtPlot *plt = plotAt( row, col );
    if ( plt )
    {
        plt->setAxisScale( axis, min, max, step );
        plt->updateAxes();
    }
}

QDomElement PlotMatrix::getDomElement( QDomDocument &doc )
{
    QDomElement element = doc.createElement("plotmatrix");

    element.setAttribute("rows", num_rows );
    element.setAttribute("columns", num_cols );

    for(int col = 0; col< num_cols; col++)
    {
        for(int row=0; row< num_rows; row++)
        {
            PlotWidget* plot = plotAt(row,col);
            QDomElement child = plot->getDomElement(doc);

            child.setAttribute("row", row);
            child.setAttribute("col", col);

            element.appendChild( child );
        }
    }

    return element;
}


void PlotMatrix::updateLayout()
{
    for ( int row = 0; row < numRows(); row++ )
    {
        alignAxes( row, QwtPlot::xTop );
        alignAxes( row, QwtPlot::xBottom );

        alignScaleBorder( row, QwtPlot::yLeft );
        alignScaleBorder( row, QwtPlot::yRight );
    }

    for ( int col = 0; col < numColumns(); col++ )
    {
        alignAxes( col, QwtPlot::yLeft );
        alignAxes( col, QwtPlot::yRight );

        alignScaleBorder( col, QwtPlot::xBottom );
        alignScaleBorder( col, QwtPlot::xTop );
    }

    this->replot();
}

void PlotMatrix::replot()
{
    for ( int row = 0; row < numRows(); row++ )
    {
        for ( int col = 0; col < numColumns(); col++ )
        {
            QwtPlot *p = plotAt( row, col );
            if ( p )
                p->replot();
        }
    }
}

void PlotMatrix::setHorizontalLink(bool linked)
{
    qDebug() << "setHorizontalLink " << linked;
    _horizontal_link = linked;
}

void PlotMatrix::maximizeHorizontalScale()
{
    for ( int row = 0; row < numRows(); row++ )
    {
        for ( int col = 0; col < numColumns(); col++ )
        {
            PlotWidget *plot = static_cast<PlotWidget*>( plotAt( row, col ) );

            QRectF bound = plot->maximumBoundingRect();
            plot->setHorizontalAxisRange( bound.left(), bound.right() );
        }
    }
    replot();
}

void PlotMatrix::maximizeVerticalScale()
{
    for ( int row = 0; row < numRows(); row++ )
    {
        for ( int col = 0; col < numColumns(); col++ )
        {
            PlotWidget *plot = static_cast<PlotWidget*>( plotAt( row, col ) );

            QRectF bound = plot->maximumBoundingRect();
            plot->setVerticalAxisRange( bound.bottom(), bound.top() );
        }
    }
    replot();
}

void PlotMatrix::swapWidgetByName(QString name_a, QString name_b)
{
    int rowA,colA,  rowB,colB, span;

    for(int i=0; i< layout->count(); i++)
    {
        QLayoutItem * item = layout->itemAt(i);

        if(dynamic_cast<QWidgetItem *>(item))   //    <-- Note! QWidgetItem, and not QWidget!
        {
            if( QString::compare(item->widget()->windowTitle(), name_a ) == 0){
                layout->getItemPosition(i, &rowA, &colA, &span, &span);
            }
            else if (QString::compare(item->widget()->windowTitle(), name_b ) == 0){
                layout->getItemPosition(i, &rowB, &colB, &span, &span);
            }
        }
    }

    swapPlots(rowA,colA,  rowB,colB);

    updateLayout();
}

void PlotMatrix::onHorizontalAxisScaleChanged(QRectF range)
{
    if( ! _horizontal_link ) return;

    for(int i=0; i< layout->count(); i++)
    {
        QLayoutItem * item = layout->itemAt(i);
        if( item  ){
            PlotWidget *widget = static_cast<PlotWidget *>( item->widget() );
            if (widget )
            {
                widget->setHorizontalAxisRange( range.left(), range.right() );
            }
        }
    }
    this->replot();
}

void PlotMatrix::alignAxes( int rowOrColumn, int axis )
{
    if ( axis == QwtPlot::yLeft || axis == QwtPlot::yRight )
    {
        double maxExtent = 0;

        for ( int row = 0; row < numRows(); row++ )
        {
            QwtPlot *p = plotAt( row, rowOrColumn );
            if ( p )
            {
                QwtScaleWidget *scaleWidget = p->axisWidget( axis );

                QwtScaleDraw *sd = scaleWidget->scaleDraw();
                sd->setMinimumExtent( 0.0 );

                const double extent = sd->extent( scaleWidget->font() );
                if ( extent > maxExtent )
                    maxExtent = extent;
            }
        }

        for ( int row = 0; row < numRows(); row++ )
        {
            QwtPlot *p = plotAt( row, rowOrColumn );
            if ( p )
            {
                QwtScaleWidget *scaleWidget = p->axisWidget( axis );
                scaleWidget->scaleDraw()->setMinimumExtent( maxExtent );
            }
        }
    }
    else
    {
        double maxExtent = 0;

        for ( int col = 0; col < numColumns(); col++ )
        {
            QwtPlot *p = plotAt( rowOrColumn, col );
            if ( p )
            {
                QwtScaleWidget *scaleWidget = p->axisWidget( axis );

                QwtScaleDraw *sd = scaleWidget->scaleDraw();
                sd->setMinimumExtent( 0.0 );

                const double extent = sd->extent( scaleWidget->font() );
                if ( extent > maxExtent )
                    maxExtent = extent;
            }
        }

        for ( int col = 0; col < numColumns(); col++ )
        {
            QwtPlot *p = plotAt( rowOrColumn, col );
            if ( p )
            {
                QwtScaleWidget *scaleWidget = p->axisWidget( axis );
                scaleWidget->scaleDraw()->setMinimumExtent( maxExtent );
            }
        }
    }
}

void PlotMatrix::alignScaleBorder( int rowOrColumn, int axis )
{
    int startDist = 0;
    int endDist = 0;

    if ( axis == QwtPlot::yLeft )
    {
        QwtPlot *p = plotAt( rowOrColumn, 0 );
        if ( p )
            p->axisWidget( axis )->getBorderDistHint( startDist, endDist );

        for ( int col = 1; col < numColumns(); col++ )
        {
            QwtPlot *p = plotAt( rowOrColumn, col );
            if ( p )
                p->axisWidget( axis )->setMinBorderDist( startDist, endDist );
        }
    }
    else if ( axis == QwtPlot::yRight )
    {
        QwtPlot *p = plotAt( rowOrColumn, numColumns() - 1 );
        if ( p )
            p->axisWidget( axis )->getBorderDistHint( startDist, endDist );

        for ( int col = 0; col < numColumns() - 1; col++ )
        {
            QwtPlot *p = plotAt( rowOrColumn, col );
            if ( p )
                p->axisWidget( axis )->setMinBorderDist( startDist, endDist );
        }
    }
    if ( axis == QwtPlot::xTop )
    {
        QwtPlot *p = plotAt( rowOrColumn, 0 );
        if ( p )
            p->axisWidget( axis )->getBorderDistHint( startDist, endDist );

        for ( int row = 1; row < numRows(); row++ )
        {
            QwtPlot *p = plotAt( row, rowOrColumn );
            if ( p )
                p->axisWidget( axis )->setMinBorderDist( startDist, endDist );
        }
    }
    else if ( axis == QwtPlot::xBottom )
    {
        QwtPlot *p = plotAt( numRows() - 1, rowOrColumn );
        if ( p )
            p->axisWidget( axis )->getBorderDistHint( startDist, endDist );

        for ( int row = 0; row < numRows() - 1; row++ )
        {
            QwtPlot *p = plotAt( row, rowOrColumn );
            if ( p )
                p->axisWidget( axis )->setMinBorderDist( startDist, endDist );
        }
    }
}
