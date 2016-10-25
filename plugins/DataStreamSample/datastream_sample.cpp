#include "datastream_sample.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QDebug>
#include <thread>
#include <mutex>
#include <chrono>
#include <thread>
#include <math.h>

DataStreamSample::DataStreamSample()
{
    QStringList  words_list;
    words_list << "siam" << "tre" << "piccoli" << "porcellin"
               << "mai" << "nessun" << "ci" << "dividera";

    foreach( const QString& name, words_list)
    {
        double A =  qrand()/(double)RAND_MAX * 6 - 3;
        double B =  qrand()/(double)RAND_MAX *3;
        double C =  qrand()/(double)RAND_MAX *3;
        double D =  qrand()/(double)RAND_MAX *2 -1;

        vect_A.push_back(A);
        vect_B.push_back(B);
        vect_C.push_back(C);
        vect_D.push_back(D);


        PlotDataPtr plot( new PlotData() );
        plot->setName( name.toStdString() );
        plot->setMaximumRangeX( 4.0 );
        _plot_data.numeric.insert( std::make_pair( name.toStdString(), plot) );
    }
    _enabled = false;
}

bool DataStreamSample::launch()
{
    _running = true;
    _thread = std::thread([this](){ this->update();} );
    return true;
}

void DataStreamSample::enableStreaming(bool enable) { _enabled = enable; }

bool DataStreamSample::isStreamingEnabled() const { return _enabled; }

DataStreamSample::~DataStreamSample()
{
    _running = false;
    if( _thread.joinable())
        _thread.join();
}



void DataStreamSample::update()
{
    static auto prev_time = std::chrono::high_resolution_clock::now();

    double t = 0;

    _running = true;
    while( _running )
    {
        size_t index=0;

        t+= 0.01;

        for (auto& it: _plot_data.numeric )
        {
            double A =  vect_A[index];
            double B =  vect_B[index];
            double C =  vect_C[index];
            double D =  vect_D[index];
            index++;

            auto& plot = it.second;

            double y =  A*sin(B*t + C) +D*t*0.02;

            if( _enabled )
                plot->pushBack( PlotData::Point( t, y ) );
        }

        prev_time += std::chrono::milliseconds(10);
        std::this_thread::sleep_until ( prev_time );
    }
}
