#include "plotwidget.h"
#include <QDebug>
#include <QDrag>
#include <QMimeData>
#include <QDragEnterEvent>
#include <qwt_plot_canvas.h>
#include <qwt_scale_engine.h>
#include <qwt_plot_layout.h>
#include <QAction>
#include <QMenu>
#include "removecurvedialog.h"

PlotWidget::PlotWidget(QWidget *parent): QwtPlot(parent)
{
    setAcceptDrops(true);
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

    removeCurveAction = new QAction(tr("&Remove curves"), this);
    removeCurveAction->setStatusTip(tr("Remove one or more curves from this plot"));
    connect(removeCurveAction, SIGNAL(triggered()), this, SLOT(launchRemoveCurveDialog()));
}

PlotWidget::~PlotWidget()
{
    detachAllCurves();
}

void PlotWidget::addCurve(const QString &name, PlotData *data)
{
    qDebug() << "attach " << name;
    QwtPlotCurve *curve = new QwtPlotCurve(name);
    _curve_list.insert( std::make_pair(name, curve));

    curve->setData( data  );
    curve->attach( this );

    curve->setPen( data->colorHint(), 1.0 );
    curve->setRenderHint( QwtPlotItem::RenderAntialiased, true );

    this->replot();
}

void PlotWidget::removeCurve(const QString &name)
{
    std::map<QString, QwtPlotCurve*>::iterator it = _curve_list.find(name);
    if( it != _curve_list.end() )
    {
         QwtPlotCurve* curve = it->second;
         curve->detach();
         this->replot();
         _curve_list.erase( it );
    }
}

bool PlotWidget::isEmpty()
{
    return this->itemList().empty();
}

void PlotWidget::dragEnterEvent(QDragEnterEvent *event)
{
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
        if( format.contains( "plot_area") )
        {
            QString source_name;
            stream >> source_name;

            if(QString::compare( windowTitle(),source_name ) != 0 ){
                event->acceptProposedAction();
            }
        }
    }
}
void PlotWidget::dragMoveEvent(QDragMoveEvent *event)
{

}


void PlotWidget::dropEvent(QDropEvent *event)
{
    const QMimeData *mimeData = event->mimeData();
    QStringList mimeFormats = mimeData->formats();

    foreach(QString format, mimeFormats)
    {
        QByteArray encoded = mimeData->data( format );
        QDataStream stream(&encoded, QIODevice::ReadOnly);

        if( format.contains( "qabstractitemmodeldatalist") )
        {
            while (!stream.atEnd())
            {
                int row, col;
                QMap<int,  QVariant> roleDataMap;

                stream >> row >> col >> roleDataMap;

                QString itemName = roleDataMap[0].toString();
                qDebug() << "curveNameDropped " << itemName;
                emit curveNameDropped( itemName , this );
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
    this->detachItems(QwtPlotItem::Rtti_PlotItem, false);

    _curve_list.erase(_curve_list.begin(), _curve_list.end());

}

QDomElement PlotWidget::getDomElement( QDomDocument &doc)
{
    QDomElement element = doc.createElement("plot");

    qDebug() << ">> add widget";
    std::map<QString, QwtPlotCurve*>::iterator it;

    for( it=_curve_list.begin(); it != _curve_list.end(); ++it)
    {
        QDomElement curve = doc.createElement("curve");
        curve.setAttribute( "name", it->first);
        curve.setNodeValue("1");
        element.appendChild(curve);
        qDebug() << ">> add curve";
    }
    return element;
}

void PlotWidget::contextMenuEvent(QContextMenuEvent *event)
{
  //  detachAllCurves();
    QMenu menu(this);
    menu.addAction(removeCurveAction);

    removeCurveAction->setEnabled( ! _curve_list.empty() );

    menu.exec( event->globalPos() );
}

void PlotWidget::launchRemoveCurveDialog()
{
    RemoveCurveDialog* dialog = new RemoveCurveDialog(this, _curve_list);
    dialog->exec();
}

void PlotWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
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
    else if(event->button() == Qt::RightButton)
    {
        qDebug() << "RightButton";
    }
}

