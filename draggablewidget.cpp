#include "draggablewidget.h"
#include <QDebug>
#include <QDrag>
#include <QMimeData>
#include <QDragEnterEvent>

DragableWidget::DragableWidget(QWidget *parent): QwtPlot(parent)
{
    setAcceptDrops(true);
}

void DragableWidget::dragEnterEvent(QDragEnterEvent *event)
{
    QString source_text = event->mimeData()->text();
    qDebug() << this->windowTitle() << "dragEnterEvent " << source_text;

    if(QString::compare( windowTitle(),source_text ) != 0 )
        event->acceptProposedAction();
}

void DragableWidget::dragMoveEvent(QDragMoveEvent *event)
{

}

void DragableWidget::dropEvent(QDropEvent *event)
{
    emit swapWidgets( event->mimeData()->text(), windowTitle() );
}

void DragableWidget::mousePressEvent(QMouseEvent *event)
{
     if (event->button() == Qt::LeftButton)
     {
        QDrag *drag = new QDrag(this);
        QMimeData *mimeData = new QMimeData;

        mimeData->setText( this->windowTitle() );
        drag->setMimeData(mimeData);

        drag->exec();
     }
}

