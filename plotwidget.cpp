#include "plotwidget.h"
#include <QDebug>
#include <QDrag>
#include <QMimeData>
#include <QDragEnterEvent>
#include <qwt_plot_canvas.h>
#include <qwt_scale_engine.h>
#include <qwt_plot_layout.h>

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
}

PlotWidget::~PlotWidget()
{
    detachAllCurves();
}

void PlotWidget::addCurve(const QString &name, PlotData *data)
{
    qDebug() << "attach " << name;
    QwtPlotCurve *curve = new QwtPlotCurve(name);

    curve->setData( data  );
    curve->attach( this );

    curve->setPen( data->colorHint(), 1.0 );
    curve->setRenderHint( QwtPlotItem::RenderAntialiased, true );

    this->replot();
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

            /// while (!stream.atEnd()) {
            //    int row, col;
            //    QMap<int,  QVariant> roleDataMap;
            //    stream >> row >> col >> roleDataMap;
           // }
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
  /*  //for(std::map<QString, QwtPlotCurve*>::iterator it = _curve_list.begin(); it != _curve_list.end(); it++)
    while(_curve_list.empty() == false )
    {
        std::map<QString, QwtPlotCurve*>::iterator it = _curve_list.begin();
         QwtPlotCurve* curve = it->second;
         curve->detach();

         // needed to avoid deleting the original data
         curve->swapData( 0 );
         _curve_list.erase( it );
    }*/
}

void PlotWidget::contextMenuEvent(QContextMenuEvent *event)
{
    detachAllCurves();
    this->replot();
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

