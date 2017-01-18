#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QCoreApplication::setApplicationName("PlotJuggler");
    QCoreApplication::setApplicationVersion("0.6.2");

    qApp->setStyleSheet(QString("QToolTip {\n"
                                "   border: 1px solid black;\n"
                                "   border-radius: 6px;\n"
                                "   background: white;\n}" ));

    MainWindow w;
    w.show();

    return app.exec();
}
