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

    virtual PlotDataMap& getDataMap() { return _plot_data; }

    virtual void enableStreaming(bool enable) { _enabled = enable; }

    virtual bool isStreamingEnabled() const { return _enabled; }


    virtual ~DataStreamSample() { _running = false; _thread.join(); }

    virtual const char* name();

private:

    void update();

    PlotDataMap _plot_data;
    bool _enabled;

    std::thread _thread;

    bool _running;

    std::vector<double> vect_A;
    std::vector<double> vect_B;
    std::vector<double> vect_C;
    std::vector<double> vect_D;

};

#endif // DATALOAD_CSV_H
