#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    qApp->setStyleSheet(QString("QToolTip {\n"
                                "   border: 1px solid black;\n"
                                "   border-radius: 6px;\n"
                                "   background: white;\n}" ));

    MainWindow w;
    w.show();

    return a.exec();
}
