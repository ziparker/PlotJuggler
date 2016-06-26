#include "datastream_ROS.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QDebug>
#include <thread>
#include <mutex>
#include <chrono>
#include <thread>
#include <ros/ros.h>
#include <ros/master.h>

#include "selectlistdialog.h"

DataStreamROS::DataStreamROS()
{
  //  _thread = std::thread([this](){ this->update();} );
    _enabled = false;
}

PlotDataMap& DataStreamROS::getDataMap()
{
    return _plot_data;
}

bool DataStreamROS::launch()
{
    _running = false;
    ros::master::setRetryTimeout( ros::WallDuration(5) );

    if( !ros::master::check() )
    {
        QMessageBox msgBox;
        msgBox.setText("ROS master is not running. Aborting.");
        msgBox.exec();

        return false;
    }

    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);

    QStringList topic_advertised;

    for (ros::master::TopicInfo topic_info: topic_infos)
    {
        topic_advertised.append( QString( topic_info.name.c_str() )  );
    }

    SelectFromListDialog dialog( &topic_advertised, false, 0 );
    int res = dialog.exec();

    return res == QDialog::Accepted;
}

void DataStreamROS::enableStreaming(bool enable) { _enabled = enable; }

bool DataStreamROS::isStreamingEnabled() const { return _enabled; }

DataStreamROS::~DataStreamROS() { _running = false; _thread.join(); }

const char*  DataStreamROS::name()
{
    return "ROS Topic Streamer";
}

void DataStreamROS::update()
{
    _running = false;
    ros::master::setRetryTimeout( ros::WallDuration(5) );

    while( !ros::master::check() )
    {
        qDebug() <<   "ROS MASTER not available. Retry in 5 seconds";
        std::this_thread::sleep_for( std::chrono::milliseconds(5000) );
    }
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);


    _running = true;
    while( _running )
    {
        std::this_thread::sleep_for( std::chrono::milliseconds(100) );
    }
}
