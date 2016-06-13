#include "subwindow.h"
#include <QDebug>

SubWindow::SubWindow(QWidget *parent) : QMainWindow(parent)
{

}

void SubWindow::closeEvent(QCloseEvent *event)
{
    qDebug() << "SubWindow::closeEvent";
    emit closeRequestedByUser();
    QMainWindow::closeEvent(event);
}
