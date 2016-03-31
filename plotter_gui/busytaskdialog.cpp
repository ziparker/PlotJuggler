#include "busytaskdialog.h"
#include <QApplication>

BusyTaskDialog::BusyTaskDialog(QString text) :
    QProgressDialog(text, "Cancel", 0, 100)
{
   setWindowFlags(Qt::FramelessWindowHint);
   setAttribute( Qt::WA_DeleteOnClose, true );
}





