#include "plotwidget.h"
#include <QDebug>
#include <QDrag>
#include <QMimeData>
#include <QDragEnterEvent>
#include <qwt_plot_canvas.h>
#include <qwt_scale_engine.h>
#include <qwt_plot_layout.h>
#include <QAction>
#include <QMessageBox>
#include <QMenu>
#include <limits>
#include "removecurvedialog.h"
#include "curvecolorpick.h"
#include <QApplication>
#include <set>

PlotWidget::PlotWidget(PlotDataMap *datamap, QWidget *parent):
    QwtPlot(parent),
    _zoomer( 0 ),
    _magnifier(0 ),
    _panner( 0 ),
    _tracker ( 0 ),
    _legend( 0 ),
    _mapped_data( datamap )
{
    this->setAcceptDrops( true );
    this->setMinimumWidth( 100 );
    this->setMinimumHeight( 100 );

    this->sizePolicy().setHorizontalPolicy( QSizePolicy::Expanding);
    this->sizePolicy().setVerticalPolicy( QSizePolicy::Expanding);

    QwtPlotCanvas *canvas = new QwtPlotCanvas(this);
    canvas->setFrameStyle( QFrame::NoFrame );
    canvas->setPaintAttribute( QwtPlotCanvas::BackingStore, false );

    this->setCanvas( canvas );
    this->setCanvasBackground( QColor( 250, 250, 250 ) );
    this->setAxisAutoScale(0, true);

    this->axisScaleEngine(QwtPlot::xBottom)->setAttribute(QwtScaleEngine::Floating,true);
    this->plotLayout()->setAlignCanvasToScales( true );

    //--------------------------
    _zoomer = ( new QwtPlotZoomer( this->canvas() ) );
    _magnifier = ( new PlotMagnifier( this->canvas() ) );
    _panner = ( new QwtPlotPanner( this->canvas() ) );
    _tracker = ( new CurveTracker( this ) );

    _zoomer->setRubberBandPen( QColor( Qt::red , 1, Qt::DotLine) );
    _zoomer->setTrackerPen( QColor( Qt::green, 1, Qt::DotLine ) );
    _zoomer->setMousePattern( QwtEventPattern::MouseSelect1, Qt::LeftButton, Qt::NoModifier );
    connect(_zoomer, SIGNAL(zoomed(QRectF)), this, SLOT(on_externallyResized(QRectF)) );

    _magnifier->setAxisEnabled(xTop, false);
    _magnifier->setAxisEnabled(yRight, false);

    // disable right button. keep mouse wheel
    _magnifier->setMouseButton( Qt::NoButton );

    connect(_magnifier, SIGNAL(rescaled(QRectF)), this, SLOT(on_externallyResized(QRectF)) );

    _panner->setMouseButton(  Qt::MiddleButton, Qt::NoModifier);

    this->canvas()->setContextMenuPolicy( Qt::ContextMenuPolicy::CustomContextMenu );
    connect( canvas, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(canvasContextMenuTriggered(QPoint)) );
    //-------------------------

    buildActions();
    buildLegend();

}

void PlotWidget::buildActions()
{
    removeCurveAction = new QAction(tr("&Remove curves"), this);
    removeCurveAction->setStatusTip(tr("Remove one or more curves from this plot"));
    connect(removeCurveAction, SIGNAL(triggered()), this, SLOT(launchRemoveCurveDialog()));

    changeColorsAction = new QAction(tr("&Change colors"), this);
    changeColorsAction->setStatusTip(tr("Change the color of the curves"));
    connect(changeColorsAction, SIGNAL(triggered()), this, SLOT(launchChangeColorDialog()));

    showPointsAction = new QAction(tr("&Show lines and points"), this);
    showPointsAction->setCheckable( true );
    showPointsAction->setChecked( false );
    connect(showPointsAction, SIGNAL(triggered(bool)), this, SLOT(on_showPoints(bool)));

    QIcon iconH;
    iconH.addFile(QStringLiteral(":/icons/resources/resize_horizontal.png"), QSize(26, 26), QIcon::Normal, QIcon::Off);

    QIcon iconV;
    iconV.addFile(QStringLiteral(":/icons/resources/resize_vertical.png"), QSize(26, 26), QIcon::Normal, QIcon::Off);

    zoomOutHorizontallyAction = new QAction(tr("&Zoom Out Horizontally"), this);
    zoomOutHorizontallyAction->setIcon(iconH);
    connect(zoomOutHorizontallyAction, SIGNAL(triggered()), this, SLOT(zoomOutHorizontal()));

    zoomOutVerticallyAction = new QAction(tr("&Zoom Out Vertically"), this);
    zoomOutVerticallyAction->setIcon(iconV);
    connect(zoomOutVerticallyAction, SIGNAL(triggered()), this, SLOT(zoomOutVertical()));

}

void PlotWidget::buildLegend()
{
    _legend = new QwtPlotLegendItem();
    _legend->attach( this );

    _legend->setRenderHint( QwtPlotItem::RenderAntialiased );
    QColor color( Qt::black );
    _legend->setTextPen( color );
    _legend->setBorderPen( color );
    QColor c( Qt::white );
    c.setAlpha( 200 );
    _legend->setBackgroundBrush( c );

    _legend->setMaxColumns( 1 );
    _legend->setAlignment( Qt::Alignment( Qt::AlignTop | Qt::AlignRight ) );
    _legend->setBackgroundMode( QwtPlotLegendItem::BackgroundMode::LegendBackground   );

    _legend->setBorderRadius( 6 );
    _legend->setMargin( 4 );
    _legend->setSpacing( 2 );
    _legend->setItemMargin( 0 );

    QFont font = _legend->font();
    font.setPointSize( 9 );
    _legend->setFont( font );
}



PlotWidget::~PlotWidget()
{
    detachAllCurves();
}

bool PlotWidget::addCurve(const QString &name, bool do_replot)
{
    PlotDataMap::iterator it = _mapped_data->find( name.toStdString() );
    if( it == _mapped_data->end())
    {
        return false;
    }

    if( _curve_list.find(name) != _curve_list.end())
    {
        return false;
    }

    PlotDataPtr data = it->second;

    QwtPlotCurve *curve = new QwtPlotCurve(name);
    _curve_list.insert( std::make_pair(name, curve));

    PlotDataQwt* plot_qwt = new PlotDataQwt( data->getVectorX(), data->getVectorY(), data->name() );

    curve->setData( plot_qwt );
    curve->attach( this );
    curve->setStyle( QwtPlotCurve::Lines);

    int red, green,blue;
    data->getColorHint(& red, &green, &blue);
    curve->setPen( QColor( red, green, blue), 1.0 );

    curve->setRenderHint( QwtPlotItem::RenderAntialiased, true );

    QRectF bounding = maximumBoundingRect();
    _magnifier->setAxisLimits( yLeft, bounding.bottom(), bounding.top());
    _magnifier->setAxisLimits( xBottom, bounding.left(), bounding.right());

    if( do_replot )
    {
        this->setAxisScale(yLeft,   bounding.bottom(), bounding.top() );
        this->setAxisScale(xBottom, bounding.left(), bounding.right() );
        replot();
    }
    return true;
}

void PlotWidget::removeCurve(const QString &name)
{
    std::map<QString, QwtPlotCurve*>::iterator it = _curve_list.find(name);
    if( it != _curve_list.end() )
    {
        QwtPlotCurve* curve = it->second;
        curve->detach();
        replot();
        _curve_list.erase( it );
    }
}

bool PlotWidget::isEmpty()
{
    return _curve_list.empty();
}

const std::map<QString, QwtPlotCurve *> &PlotWidget::curveList()
{
    return _curve_list;
}

void PlotWidget::dragEnterEvent(QDragEnterEvent *event)
{
    QwtPlot::dragEnterEvent(event);

    const QMimeData *mimeData = event->mimeData();
    QStringList mimeFormats = mimeData->formats();
    foreach(QString format, mimeFormats)
    {
        QByteArray encoded = mimeData->data( format );
        QDataStream stream(&encoded, QIODevice::ReadOnly);

        if( format.contains( "qabstractitemmodeldatalist") )
        {
            event->acceptProposedAction();
        }
        if( format.contains( "plot_area")  )
        {
            QString source_name;
            stream >> source_name;

            if(QString::compare( windowTitle(),source_name ) != 0 ){
                event->acceptProposedAction();
            }
        }
    }
}
void PlotWidget::dragMoveEvent(QDragMoveEvent *)
{

}


void PlotWidget::dropEvent(QDropEvent *event)
{
    QwtPlot::dropEvent(event);

    const QMimeData *mimeData = event->mimeData();
    QStringList mimeFormats = mimeData->formats();

    foreach(QString format, mimeFormats)
    {
        QByteArray encoded = mimeData->data( format );
        QDataStream stream(&encoded, QIODevice::ReadOnly);

        if( format.contains( "qabstractitemmodeldatalist") )
        {
            bool plot_added = false;
            while (!stream.atEnd())
            {
                int row, col;
                QMap<int,  QVariant> roleDataMap;

                stream >> row >> col >> roleDataMap;

                QString curve_name = roleDataMap[0].toString();
                addCurve( curve_name );
                plot_added = true;
            }
            if( plot_added )
            {
                emit plotModified();
            }
        }
        if( format.contains( "plot_area") )
        {
            QString source_name;
            stream >> source_name;
            emit swapWidgets( source_name, windowTitle() );
        }
    }
}

void PlotWidget::detachAllCurves()
{
    this->detachItems(QwtPlotItem::Rtti_PlotCurve, false);
    _curve_list.erase(_curve_list.begin(), _curve_list.end());
}

QDomElement PlotWidget::xmlSaveState( QDomDocument &doc)
{
    QDomElement plot_el = doc.createElement("plot");

    QDomElement range_el = doc.createElement("range");
    QRectF rect = this->currentBoundingRect();
    range_el.setAttribute("bottom", rect.bottom());
    range_el.setAttribute("top", rect.top());
    range_el.setAttribute("left", rect.left());
    range_el.setAttribute("right", rect.right());
    plot_el.appendChild(range_el);

    std::map<QString, QwtPlotCurve*>::iterator it;

    for( it=_curve_list.begin(); it != _curve_list.end(); ++it)
    {
        QString name = it->first;
        QwtPlotCurve* curve = it->second;
        QDomElement curve_el = doc.createElement("curve");
        curve_el.setAttribute( "name",name);
        curve_el.setAttribute( "R", curve->pen().color().red());
        curve_el.setAttribute( "G", curve->pen().color().green());
        curve_el.setAttribute( "B", curve->pen().color().blue());

        plot_el.appendChild(curve_el);
    }
    return plot_el;
}

bool PlotWidget::xmlLoadState(QDomElement &plot_widget, QMessageBox::StandardButton* answer)
{
    QDomElement curve;
    std::set<QString> added_curve_names;

    for (  curve = plot_widget.firstChildElement( "curve" )  ;
           !curve.isNull();
           curve = curve.nextSiblingElement( "curve" ) )
    {
        QString curve_name = curve.attribute("name");
        int R = curve.attribute("R").toInt();
        int G = curve.attribute("G").toInt();
        int B = curve.attribute("B").toInt();
        QColor color(R,G,B);

        bool added_to_list = addCurve(curve_name, false);
        if( added_to_list )
        {
            _curve_list[curve_name]->setPen( color, 1.0);
            added_curve_names.insert(curve_name );
        }
        else{
            if( *answer !=  QMessageBox::YesToAll)
            {
                *answer = QMessageBox::question(0,
                                               tr("Warning"),
                                               tr("Can't find the curve with name %1.\n Do you want to ignore it? ").arg(curve_name),
                                               QMessageBox::Yes | QMessageBox::YesToAll | QMessageBox::Abort ,
                                               QMessageBox::Abort );
            }

            if( *answer ==  QMessageBox::Yes || *answer ==  QMessageBox::YesToAll) {
                continue;
            }

            if( *answer ==  QMessageBox::Abort) {
                return false;
            }
        }
    }

    std::map<QString, QwtPlotCurve*>::iterator it;
    for( it = _curve_list.begin(); it != _curve_list.end(); it++)
    {
        QString curve_name = it->first;
        if( added_curve_names.find( curve_name ) == added_curve_names.end())
        {
            removeCurve( curve_name );
        }
    }

    QDomElement rectangle =  plot_widget.firstChildElement( "range" );
    QRectF rect;
    rect.setBottom( rectangle.attribute("bottom").toDouble());
    rect.setTop( rectangle.attribute("top").toDouble());
    rect.setLeft( rectangle.attribute("left").toDouble());
    rect.setRight( rectangle.attribute("right").toDouble());
    this->setScale( rect, false);

    return true;
}


QRectF PlotWidget::currentBoundingRect()
{
    QRectF rect;
    rect.setBottom( this->canvasMap( yLeft ).s1() );
    rect.setTop( this->canvasMap( yLeft ).s2() );

    rect.setLeft( this->canvasMap( xBottom ).s1() );
    rect.setRight( this->canvasMap( xBottom ).s2() );
    return rect;
}

CurveTracker *PlotWidget::tracker()
{
    return _tracker;
}

void PlotWidget::setScale(QRectF rect, bool emit_signal)
{
    this->setAxisScale( yLeft, rect.bottom(), rect.top());
    this->setAxisScale( xBottom, rect.left(), rect.right());

    if( emit_signal) {
        emit rectChanged(this, rect);
    }
}


void PlotWidget::setAxisScale(int axisId, double min, double max, double step)
{
    if (axisId == xBottom)
    {
        std::map<QString, QwtPlotCurve*>::iterator it;
        for(it = _curve_list.begin(); it != _curve_list.end(); ++it)
        {
            PlotDataQwt* data = static_cast<PlotDataQwt*>( it->second->data() );
            data->setRangeX( min,max );
        }
    }

    QwtPlot::setAxisScale( axisId, min, max, step);
}

QRectF PlotWidget::maximumBoundingRect()
{
    float left   = std::numeric_limits<float>::max();
    float right  = std::numeric_limits<float>::min();

    float bottom = std::numeric_limits<float>::max();
    float top    = std::numeric_limits<float>::min();

    std::map<QString, QwtPlotCurve*>::iterator it;
    for(it = _curve_list.begin(); it != _curve_list.end(); ++it)
    {
        PlotDataQwt* data = static_cast<PlotDataQwt*>( it->second->data() );
        QRectF bounding_rect = data->maximumBoundingRect();

        if( bottom > bounding_rect.top() )    bottom = bounding_rect.top();
        if( top < bounding_rect.bottom() )    top = bounding_rect.bottom();

        if( left > bounding_rect.left() )    left = bounding_rect.left();
        if( right < bounding_rect.right() ) right = bounding_rect.right();
    }

    if( fabs(top-bottom) < 1e-10  )
    {
        return QRectF(left, top+0.01,  right - left, -0.02 ) ;
    }
    else{
        return QRectF(left, top,  right - left, bottom - top ) ;
    }
}


void PlotWidget::replot()
{
    if( _zoomer )
        _zoomer->setZoomBase( false );

    // QRectF canvas_range = currentBoundingRect();

    if(_tracker ) {
        // _tracker->onExternalZoom( canvas_range );
        _tracker->refreshPosition( );
    }

    QwtPlot::replot();


    //_prev_bounding = canvas_range;
}

void PlotWidget::launchRemoveCurveDialog()
{
    RemoveCurveDialog* dialog = new RemoveCurveDialog(this);
    unsigned prev_curve_count = _curve_list.size();

    std::map<QString, QwtPlotCurve*>::iterator it;
    for(it = _curve_list.begin(); it != _curve_list.end(); ++it)
    {
        dialog->addCurveName( it->first );
    }

    dialog->exec();

    if( prev_curve_count != _curve_list.size() )
    {
        emit plotModified();
    }
}

void PlotWidget::launchChangeColorDialog()
{
    std::map<QString,QColor> color_by_name;

    std::map<QString, QwtPlotCurve*>::iterator it;
    for(it = _curve_list.begin(); it != _curve_list.end(); ++it)
    {
        const QString& curve_name = it->first;
        QwtPlotCurve* curve = it->second;
        color_by_name.insert(std::make_pair( curve_name, curve->pen().color() ));
    }

    CurveColorPick* dialog = new CurveColorPick(&color_by_name, this);
    dialog->exec();

    bool modified = false;

    for(it = _curve_list.begin(); it != _curve_list.end(); ++it)
    {
        const QString& curve_name = it->first;
        QwtPlotCurve* curve = it->second;
        QColor new_color = color_by_name[curve_name];
        if( curve->pen().color() != new_color)
        {
            curve->setPen( color_by_name[curve_name], 1.0 );
            modified = true;
        }
    }
    if( modified){
        emit plotModified();
    }
}

void PlotWidget::on_showPoints(bool checked)
{
    std::map<QString, QwtPlotCurve*>::iterator it;
    for(it = _curve_list.begin(); it != _curve_list.end(); ++it)
    {
        QwtPlotCurve* curve = it->second;
        if( checked )
        {
            curve->setStyle( QwtPlotCurve::LinesAndDots);
        }
        else{
            curve->setStyle( QwtPlotCurve::Lines);
        }
    }
}

void PlotWidget::on_externallyResized(QRectF rect)
{
    emit rectChanged( this, rect);
}

void PlotWidget::zoomOutHorizontal()
{
    QRectF act = currentBoundingRect();
    QRectF max = maximumBoundingRect();
    act.setLeft( max.left() );
    act.setRight( max.right() );
    this->setScale(act);
    qDebug() << act;
}

void PlotWidget::zoomOutVertical()
{
    QRectF act = currentBoundingRect();
    QRectF max = maximumBoundingRect();
    act.setTop( max.top() );
    act.setBottom( max.bottom() );
    this->setScale(act);
    qDebug() << act;
}

void PlotWidget::canvasContextMenuTriggered(const QPoint &pos)
{
    QMenu menu(this);
    menu.addAction(removeCurveAction);
    menu.addAction(changeColorsAction);
    menu.addAction(showPointsAction);

    menu.addAction(zoomOutHorizontallyAction);
    menu.addAction(zoomOutVerticallyAction);

    removeCurveAction->setEnabled( ! _curve_list.empty() );
    changeColorsAction->setEnabled(  ! _curve_list.empty() );

    menu.exec( canvas()->mapToGlobal(pos) );

}

void PlotWidget::mousePressEvent(QMouseEvent *event)
{
    if( event->button() == Qt::LeftButton)
    {
        if (event->modifiers() & Qt::ControlModifier )
        {
            QDrag *drag = new QDrag(this);
            QMimeData *mimeData = new QMimeData;

            QByteArray data;
            QDataStream dataStream(&data, QIODevice::WriteOnly);

            dataStream << this->windowTitle();

            mimeData->setData("plot_area", data );
            drag->setMimeData(mimeData);
            drag->exec();
        }
        else if( event->modifiers() == Qt::NoModifier)
        {
            QApplication::setOverrideCursor(QCursor(QPixmap(":/icons/resources/zoom_in_32px.png")));
        }
    }

    if( event->button() == Qt::MiddleButton && event->modifiers() == Qt::NoModifier)
    {
        QApplication::setOverrideCursor(QCursor(QPixmap(":/icons/resources/move.png")));
    }

    QwtPlot::mousePressEvent(event);
}

void PlotWidget::mouseReleaseEvent(QMouseEvent *event )
{
    QApplication::restoreOverrideCursor();
    QwtPlot::mouseReleaseEvent(event);
}

