#ifndef DATASTREAM_SAMPLE_CSV_H
#define DATASTREAM_SAMPLE_CSV_H

#include <QtPlugin>
#include <thread>
#include "PlotJuggler/datastreamer_base.h"


class  DataStreamSample: public QObject, DataStreamer
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.icarustechnology.Superplotter.DataStreamer" "../datastreamer.json")
    Q_INTERFACES(DataStreamer)

public:

    DataStreamSample();

    virtual PlotDataMap& getDataMap() override { return _plot_data; }

    virtual bool launch() override;

    virtual void enableStreaming(bool enable) override;

    virtual bool isStreamingEnabled() const override;

    virtual ~DataStreamSample();

    virtual const char* name() const override { return "DataStreamer Dummy"; }

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
