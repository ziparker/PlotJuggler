#include "datastream_sample.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QDebug>
#include <thread>
#include <mutex>

DataStreamSample::DataStreamSample()
{
    _enabled = false;

    QStringList  words_list;
    words_list << "siam" << "tre" << "piccoli" << "porcellin"
               << "mai" << "nessun" << "ci" << "dividera";

    long SIZE = 10*1000;


    foreach( const QString& name, words_list)
    {
        double t = 0;

        float A =  qrand()/(float)RAND_MAX * 6 - 3;
        float B =  qrand()/(float)RAND_MAX *3;
        float C =  qrand()/(float)RAND_MAX *3;
        float D =  qrand()/(float)RAND_MAX *2 -1;

        vect_A.push_back(A);
        vect_B.push_back(B);
        vect_C.push_back(C);
        vect_D.push_back(D);

        PlotDataPtr plot ( new PlotData( name.toStdString().c_str() ) );
        plot->setCapacity( SIZE );

        for (int indx=0; indx<SIZE; indx++)
        {
            t += 0.001;
            plot->pushBack( t, A*sin(B*t + C) +D*t*0.02 ) ;
        }

        _plot_data.insert( std::make_pair( name.toStdString(), plot) );
    }

  //  _thread = std::thread([this](){ this->update();} );
}

void DataStreamSample::update()
{
  /*  PlotDataMap::iterator it;

    while( 1 )
    {
        int i=0;
        for ( it =_plot_data.begin(); it != _plot_data.end(); it++)
        {


            float A =  vect_A[i];
            float B =  vect_A[i];
            float C =  vect_A[i];
            float D =  vect_A[i];
            i++;

            //double t = vectX->back() + 0.001;
         //   double y =  A*sin(B*t + C) +D*t*0.02;

          //  it->second->pushBack( t, y);
        }
        usleep( 1000 );
    }
*/
}
