#ifndef DATASTREAM_SAMPLE_CSV_H
#define DATASTREAM_SAMPLE_CSV_H

#include <QtPlugin>
#include <thread>
#include "../datastreamer_base.h"


class  DataStreamSample: public QObject, DataStreamer
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.icarustechnology.Superplotter.DataStreamer" "../datastreamer.json")
    Q_INTERFACES(DataStreamer)

public:

    DataStreamSample();

<<<<<<< HEAD
=======
    void update();

>>>>>>> b821eb0b426188e0e1635851b6eb4b29d82b171e
    virtual PlotDataMap& getDataMap() { return _plot_data; }

    virtual void enableStreaming(bool enable) { _enabled = enable; }

    virtual bool isStreamingEnabled() const { return _enabled; }

<<<<<<< HEAD
    virtual ~DataStreamSample() { _running = false; _thread.join(); }

    virtual const char* name();

private:

    void update();
=======
    virtual ~DataStreamSample() {}


    virtual std::mutex& mutex() { return _mutex;}

private:

    std::mutex _mutex;
>>>>>>> b821eb0b426188e0e1635851b6eb4b29d82b171e

    PlotDataMap _plot_data;
    bool _enabled;

    std::thread _thread;
<<<<<<< HEAD
    bool _running;
=======
>>>>>>> b821eb0b426188e0e1635851b6eb4b29d82b171e

    std::vector<double> vect_A;
    std::vector<double> vect_B;
    std::vector<double> vect_C;
    std::vector<double> vect_D;


};

#endif // DATALOAD_CSV_H
