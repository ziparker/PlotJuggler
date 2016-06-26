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
#include <QProgressDialog>
#include <QtGlobal>


#include "selectlistdialog.h"
#include <ros/callback_queue.h>


DataStreamROS::DataStreamROS()
{
    //  _thread = std::thread([this](){ this->update();} );
    _enabled = false;
}

PlotDataMap& DataStreamROS::getDataMap()
{
    return _plot_data;
}

void DataStreamROS::topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string &topic_name)
{
    using namespace RosTypeParser;

    static std::set<std::string> registered_type;

    auto& data_type = msg->getDataType();

    if( registered_type.find( data_type ) == registered_type.end() )
    {
        registered_type.insert( data_type );
        parseRosTypeDescription( data_type,
                                 msg->getMessageDefinition(),
                                 &_ros_type_map);
    }

    //------------------------------------
    uint8_t buffer[1024*16];
    double msg_time = (ros::Time::now() - _initial_time).toSec();

    RosTypeFlat flat_container;

    ros::serialization::OStream stream(buffer, sizeof(buffer));
    msg->write(stream);

    uint8_t* buffer_ptr = buffer;

    String datatype( data_type.data(), data_type.size() );
    String topicname( topic_name.data(), topic_name.length() );

    std::vector<SubstitutionRule> rules;

    buildRosFlatType( _ros_type_map, datatype, topicname, &buffer_ptr,  &flat_container);
    applyNameTransform( rules, &flat_container );

   // qDebug() << " pushing " << msg_time;

    for(auto& it: flat_container.value_renamed )
    {
        std::string field_name ( it.first.data(), it.first.size());
        auto value = it.second;

        auto plot = _plot_data.numeric.find( field_name );
        if( plot == _plot_data.numeric.end() )
        {
            PlotDataPtr temp(new PlotData());
            temp->setCapacity( 1000 );
            temp->setMaximumRangeX( 4.0 );
            auto res = _plot_data.numeric.insert( std::make_pair(field_name, temp ) );
            plot = res.first;
        }

        plot->second->pushBack( PlotData::Point(msg_time, value));
    }
}

void DataStreamROS::extractInitialSamples()
{
    _initial_time = ros::Time::now();

    int wait_time = 4;

    QProgressDialog progress_dialog;
    progress_dialog.setLabelText( "Collecting ROS topic samples to understand data layout. ");
    progress_dialog.setRange(0, wait_time*1000);
    progress_dialog.setAutoClose(true);
    progress_dialog.setAutoReset(true);

    progress_dialog.show();

    using namespace std::chrono;
    auto start_time = system_clock::now();


    while ( system_clock::now() - start_time < seconds(wait_time) )
    {
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
        int i = duration_cast<milliseconds>(system_clock::now() - start_time).count() ;
        progress_dialog.setValue( i );
        QApplication::processEvents();
        if( progress_dialog.wasCanceled() )
        {
            break;
        }
    }
    if( progress_dialog.wasCanceled() == false )
    {
        progress_dialog.cancel();
    }
}

bool DataStreamROS::rosInit()
{
    int argc = 0;
    char **argv = NULL;

    if( qgetenv("ROS_MASTER_URI").isEmpty() )
    {
        QMessageBox msgBox;
        msgBox.setText("ROS_MASTER_URI is not defined in the environment.\n"
                       "Either type the following or (preferrably) add this to your ~/.bashrc \n"
                       "file in order set up your local machine as a ROS master:\n\n"
                       "    export ROS_MASTER_URI=http://localhost:11311\n\n"
                       "Then, type 'roscore' in another shell to actually launch the master program.");
        msgBox.exec();
        return false;
    }

    try{
        ros::init(argc, argv, "superplotter_ros_streamer", ros::init_options::NoSigintHandler);

        if( !ros::master::check() )
        {
            QMessageBox msgBox;
            msgBox.setText("ROS master is not running. Aborting.");
            msgBox.exec();
            return false;
        }
    }
    catch(...)
    {
        return false;
    }
    return true;
}

bool DataStreamROS::launch()
{
    if( ! rosInit() ) return false;

    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);

    QStringList topic_advertised;

    for (ros::master::TopicInfo topic_info: topic_infos)
    {
        topic_advertised.append( QString( topic_info.name.c_str() )  );
    }

    SelectFromListDialog dialog( &topic_advertised, false, 0 );
    int res = dialog.exec();

    auto indexes = dialog.getSelectedRowNumber();

    if( res != QDialog::Accepted )
    {
        return false;
    }

    _node = std::unique_ptr<ros::NodeHandle> ( new  ros::NodeHandle() );

    for (int i=0; i<indexes.size(); i++ )
    {
        std::string topic_name =  topic_advertised.at( indexes[i] ).toStdString();
        boost::function<void(const topic_tools::ShapeShifter::ConstPtr&) > callback;
        callback = boost::bind( &DataStreamROS::topicCallback, this, _1, topic_name ) ;
        _subscribers.push_back( _node->subscribe( topic_name, 1000,  callback)  );
    }

    extractInitialSamples();

    _running = true;
    _thread = std::thread([this](){ this->update();} );

    return true;
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
    while (ros::ok() && _running)
    {
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    }
    ros::shutdown();
}
