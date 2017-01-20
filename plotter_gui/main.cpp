#include "mainwindow.h"
#include <QApplication>
#include <QSplashScreen>
#include <QThread>


QString getFunnySubtitle(){

  qsrand(time(NULL));
  int n = qrand() % 18;
  switch(n)
  {
  case 0: return "The best excuse to buy a second monitor";
  case 1: return "Now with 100% more splashscreens";
  case 2: return "Because command line tools suck";
  case 3: return "Data at your fingertips";
  case 4: return "You might have used Kst2";
  case 5: return "Have you starred me on Github?";
  case 6: return "Insert [useless splashscreen] here";
  case 7: return "Data, data everywhere";
  case 8: return "Plots at the speed of mouse";
  case 9: return "Just Plot It!";
  case 10: return "More than you can plot";
  }
  return "Juggle with data";
}

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  QCoreApplication::setApplicationName("PlotJuggler");
  QCoreApplication::setApplicationVersion("0.7.0");

  qApp->setStyleSheet(QString("QToolTip {\n"
                              "   border: 1px solid black;\n"
                              "   border-radius: 6px;\n"
                              "   background: white;\n}" ));

  QPixmap main_pixmap(":/splash/resources/splash.jpg");

  QPainter painter;
  painter.begin( &main_pixmap);
  painter.setPen(QColor(50, 50, 50));

  QString subtitle = getFunnySubtitle();
  int font_size = 36;
  do{
      painter.setFont( QFont("Arial", font_size-- ) );
  }while( painter.fontMetrics().width(subtitle) > 550 );

  painter.drawText( QRect(50, 200, 580, 100), Qt::AlignHCenter | Qt::AlignVCenter, subtitle );
  painter.end();

  QSplashScreen splash(main_pixmap);
  splash.show();

  MainWindow w;

  for (int i =0; i<(25 + subtitle.size()/2) && !splash.isHidden(); i++ ) {
    app.processEvents();
    QThread::msleep(100);
    splash.raise();
  }

  splash.close();

  w.show();

  return app.exec();
}
