#ifndef DATASTREAM_ROS_TOPIC_H
#define DATASTREAM_ROS_TOPIC_H

#include <QtPlugin>
#include <thread>
#include "topic_tools/shape_shifter.h"
#include "../datastreamer_base.h"
#include "ros-type-parser.h"

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

    bool rosInit();

    void topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string& topic_name);

    void update();

    PlotDataMap _plot_data;

    bool _enabled;

    bool _running;

    std::thread _thread;

    RosTypeParser::RosTypeMap _ros_type_map;

    void extractInitialSamples();

    ros::Time _initial_time;

    std::vector<ros::Subscriber> _subscribers;

    std::unique_ptr<ros::NodeHandle> _node;

};

#endif // DATALOAD_CSV_H
