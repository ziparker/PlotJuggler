#include "datastream_sample.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QDebug>
#include <thread>
#include <mutex>
#include <chrono>
#include <thread>

DataStreamSample::DataStreamSample()
{
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
        _plot_data.numeric.insert( std::make_pair( name.toStdString(), plot) );
    }

    _thread = std::thread([this](){ this->update();} );
    _enabled = false;
}

const char*  DataStreamSample::name()
{
    return "DataStreamerDummy";
}

void DataStreamSample::update()
{
    static auto prev_time = std::chrono::high_resolution_clock::now();

    _running = true;
    while( _running )
    {
        int index=0;

        for (auto& it: _plot_data.numeric )
        {
            float A =  vect_A[index];
            float B =  vect_B[index];
            float C =  vect_C[index];
            float D =  vect_D[index];
            index++;

            double t = 0;

            auto& plot = it.second;

            if( plot->size() > 0)
                t = plot->getRangeX().max + 0.01;

            double y =  A*sin(B*t + C) +D*t*0.02;

            if( _enabled )
                plot->pushBack( PlotData::Point( t, y ) );
        }

        prev_time += std::chrono::milliseconds(10);
        std::this_thread::sleep_until ( prev_time );
    }
}
