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

    foreach( const QString& name, words_list)
    {
        float A =  qrand()/(float)RAND_MAX * 6 - 3;
        float B =  qrand()/(float)RAND_MAX *3;
        float C =  qrand()/(float)RAND_MAX *3;
        float D =  qrand()/(float)RAND_MAX *2 -1;

        vect_A.push_back(A);
        vect_B.push_back(B);
        vect_C.push_back(C);
        vect_D.push_back(D);

        PlotDataPtr plot( new PlotData() );
        plot->setName( name.toStdString() );
        plot->setMaximumRangeX( 4.0 );
        _plot_data.insert( std::make_pair( name.toStdString(), plot) );
    }

    _thread = std::thread([this](){ this->update();} );
    _enabled = true;
}

const char*  DataStreamSample::name()
{
    return "DataStreamerDummy";
}

void DataStreamSample::update()
{
    _running = true;
    while( _running )
    {
        int i=0;

        PlotDataMap::iterator it;

        for ( it =_plot_data.begin(); it != _plot_data.end(); it++)
        {
            float A =  vect_A[i];
            float B =  vect_A[i];
            float C =  vect_A[i];
            float D =  vect_A[i];
            i++;

            double t = 0;

            auto& plot = it->second;

            if( plot->size() > 0)
                t = plot->getRangeX().max + 0.001;

            double y =  A*sin(B*t + C) +D*t*0.02;

            if( _enabled)
                plot->pushBack( t, y);
        }
        usleep( 1000 );
    }
}
