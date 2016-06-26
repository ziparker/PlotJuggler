#ifndef DATASTREAM_ROS_TOPIC_H
#define DATASTREAM_ROS_TOPIC_H

#include <QtPlugin>
#include <thread>
#include "../datastreamer_base.h"


class  DataStreamROS: public QObject, DataStreamer
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.icarustechnology.Superplotter.DataStreamer" "../datastreamer.json")
    Q_INTERFACES(DataStreamer)

public:

    DataStreamROS();

    virtual PlotDataMap &getDataMap();

    virtual bool launch();

    virtual void enableStreaming(bool enable);

    virtual bool isStreamingEnabled() const;

    virtual ~DataStreamROS();

    virtual const char* name();

private:

    void update();

    PlotDataMap _plot_data;

    bool _enabled;

    bool _running;

    std::thread _thread;


};

#endif // DATALOAD_CSV_H
