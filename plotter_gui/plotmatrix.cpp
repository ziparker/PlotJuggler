#include <qlayout.h>
#include <qpen.h>
#include <qwt_plot.h>
#include <qwt_plot_canvas.h>
#include <qwt_scale_widget.h>
#include <qwt_scale_draw.h>
#include "plotmatrix.h"
#include "customtracker.h"



PlotMatrix::PlotMatrix(PlotDataQwtMap *datamap, QWidget *parent ):
    QFrame( parent ),
    _mapped_data(datamap)
{
    num_rows = 0;
    num_cols = 0;
    widget_uid = 0;

    layout = new QGridLayout( this );

    _horizontal_link = true;
    updateLayout();
}

PickerTrackerMachine* sm = new PickerTrackerMachine();

void PlotMatrix::rebuildWidgetList()
{
    _widget_list.clear();
    _widget_list.reserve( numRows()*numColumns() );

    for ( int row = 0; row < numRows(); row++ )
    {
        for ( int col = 0; col < numColumns(); col++ )
        {
            PlotWidget *plot = static_cast<PlotWidget*>( plotAt( row, col ) );
            _widget_list.push_back( plot );
        }
    }
}

PlotWidget* PlotMatrix::addPlotWidget(int row, int col)
{
    PlotWidget *plot = new PlotWidget( _mapped_data, this );

    plot->setWindowTitle(QString("PlotWidget ") + QString::number(widget_uid++));

    CurveTracker* tracker = plot->tracker();
    tracker->setRubberBandPen( QPen( "MediumOrchid" ) );


    connect( plot, SIGNAL(swapWidgets(QString,QString)), this, SLOT(swapWidgetByName(QString,QString)) );
    connect( plot, SIGNAL(rectChanged(PlotWidget*,QRectF)), this, SLOT(on_singlePlotScaleChanged(PlotWidget*,QRectF)));

    plot->setAttribute(Qt::WA_DeleteOnClose);

    for(int i=0; i< layout->count(); i++)
    {
        QLayoutItem * item = layout->itemAt(i);
        if( item  ){
            PlotWidget *other_plot = static_cast<PlotWidget *>( item->widget() );
            if (other_plot )
            {
                connect(other_plot->tracker(),SIGNAL( moved(const QPointF&)),
                        plot->tracker(), SLOT(manualMove(const QPointF&)) );

                connect(plot->tracker(),SIGNAL(moved(const QPointF&)),
                        other_plot->tracker(), SLOT(manualMove(const QPointF&)) );
            }
        }
    }

    plot->tracker()->setEnabled( false );

    layout->addWidget( plot, row, col );

    emit plotAdded(plot);
    return plot;
}

void PlotMatrix::addRow()
{
    if( num_rows==0 && num_cols==0 )
    {
        addPlotWidget( 0, 0 );
        num_rows = 1;
        num_cols = 1;
    }
    else{
        for ( int col = 0; col < numColumns(); col++ )
        {
            addPlotWidget( num_rows, col );
        }
        num_rows++;
    }
    rebuildWidgetList();

    updateLayout();
}

void PlotMatrix::addColumn()
{
    if( num_rows==0 && num_cols==0 )
    {
        addPlotWidget( 0, 0 );
        num_rows = 1;
        num_cols = 1;
    }
    else {
        for ( int row = 0; row < numRows(); row++ )
        {
            addPlotWidget( row, num_cols );
        }
        num_cols++;
    }
    rebuildWidgetList();
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
    if( num_cols == 0){
        num_rows = 0;
    }
    rebuildWidgetList();
    updateLayout();
}

void PlotMatrix::removeRow(int row_to_delete)
{
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
    if( num_rows == 0){
        num_cols = 0;
    }
    rebuildWidgetList();
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

QDomElement PlotMatrix::xmlSaveState( QDomDocument &doc )
{
    QDomElement element = doc.createElement("plotmatrix");

    element.setAttribute("rows", num_rows );
    element.setAttribute("columns", num_cols );

    for(int col = 0; col< num_cols; col++)
    {
        for(int row=0; row< num_rows; row++)
        {
            PlotWidget* plot = plotAt(row,col);
            QDomElement child = plot->xmlSaveState(doc);

            child.setAttribute("row", row);
            child.setAttribute("col", col);

            element.appendChild( child );
        }
    }
    return element;
}

void PlotMatrix::xmlLoadState( QDomElement &plotmatrix )
{
    if( !plotmatrix.hasAttribute("rows") || !plotmatrix.hasAttribute("columns") )
    {
        qWarning() << "No [rows] or [columns] attribute in <plotmatrix> XML file!";
        return ;
    }
    int rows = plotmatrix.attribute("rows").toInt();
    int cols = plotmatrix.attribute("columns" ).toInt();

    while( rows > num_rows){ addRow(); }
    while( rows < num_rows){ removeRow( num_rows-1 );  }

    while( cols > num_cols){ addColumn();  }
    while( cols < num_cols){ removeColumn( num_cols-1 ); }

    QDomElement plot_element;
    for (  plot_element = plotmatrix.firstChildElement( "plot" )  ;
           !plot_element.isNull();
           plot_element = plot_element.nextSiblingElement( "plot" ) )
    {
        if( !plot_element.hasAttribute("row") || !plot_element.hasAttribute("col") )
        {
            qWarning() << "No [row] or [col] attribute in <plot> XML file!";
            return ;
        }
        int row = plot_element.attribute("row").toInt();
        int col = plot_element.attribute("col").toInt();

        plotAt(row,col)->xmlLoadState( plot_element );
    }
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
    for ( unsigned i = 0; i<_widget_list.size(); i++ )
    {
        PlotWidget *plot = _widget_list.at(i);
        plot->replot();
    }
}

void PlotMatrix::setHorizontalLink(bool linked)
{
    qDebug() << "setHorizontalLink " << linked;
    _horizontal_link = linked;
}

void PlotMatrix::setActiveTracker(bool active)
{
    for ( unsigned i = 0; i<_widget_list.size(); i++ )
    {
        PlotWidget *plot = _widget_list.at(i);
        plot->tracker()->setEnabled( active );
    }
}

const std::vector<PlotWidget *> &PlotMatrix::widgetList()
{
    return _widget_list;
}

void PlotMatrix::maximizeHorizontalScale()
{
    for ( unsigned i = 0; i<_widget_list.size(); i++ )
    {
        PlotWidget *plot = _widget_list.at(i);
        if( plot->isEmpty() == false)
        {
            QRectF bound_max = plot->maximumBoundingRect();
            QRectF bound_act= plot->currentBoundingRect();
            bound_act.setLeft( bound_max.left() );
            bound_act.setRight( bound_max.right() );
            plot->setScale( bound_act );
        }
    }
    replot();
}

void PlotMatrix::maximizeVerticalScale()
{
    for ( unsigned i = 0; i<_widget_list.size(); i++ )
    {
        PlotWidget *plot = _widget_list.at(i);
        if( plot->isEmpty() == false)
        {
            QRectF bound_max = plot->maximumBoundingRect();
            QRectF bound_act= plot->currentBoundingRect();
            bound_act.setBottom( bound_max.bottom() );
            bound_act.setTop( bound_max.top() );
            plot->setScale( bound_act );
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

    emit layoutModified();
}

void PlotMatrix::on_singlePlotScaleChanged(PlotWidget *modified_plot, QRectF new_range)
{
    for ( unsigned i = 0; i<_widget_list.size(); i++ )
    {
        PlotWidget *plot = _widget_list.at(i);
        if( plot->isEmpty() == false && modified_plot != plot)
        {
            QRectF bound_act = plot->currentBoundingRect();

            if( _horizontal_link )
            {
                bound_act.setLeft( new_range.left() );
                bound_act.setRight( new_range.right() );
            }
            plot->setScale( bound_act, false );
        }
    }
    replot();
    emit layoutModified();
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
