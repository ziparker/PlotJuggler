#include "subwindow.h"
#include <QDebug>

SubWindow::SubWindow(PlotDataMap *mapped_data, QWidget *parent) :
  QMainWindow(parent),
  tabbed_widget_( mapped_data, this )
{
    this->setCentralWidget( &tabbed_widget_ );
}

void SubWindow::closeEvent(QCloseEvent *event)
{
    qDebug() << "SubWindow::closeEvent";
    emit closeRequestedByUser();
    QMainWindow::closeEvent(event);
}
