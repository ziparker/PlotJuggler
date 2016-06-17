#include "dataload_ros.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include "selectxaxisdialog.h"
#include <QDebug>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "ros-type-parser.h"

DataLoadROS::DataLoadROS()
{
    _extensions.push_back( "bag");
}

const std::vector<const char*> &DataLoadROS::compatibleFileExtensions() const
{
    return _extensions;
}

int DataLoadROS::parseHeader(QFile *file,
                             std::vector<std::pair<bool,QString> >& ordered_names,
                             std::function<void(int)> updateCompletion)
{

    return 0;
}

PlotDataMap DataLoadROS::readDataFromFile(QFile *file,
                                          std::function<void(int)> updateCompletion,
                                          std::function<bool()> checkInterruption,
                                          int time_index )
{

    using namespace RosFlatTypeParser;

    QStringList all_topic_names;
    PlotDataMap plot_data;

    rosbag::Bag bag;
    bag.open( file->fileName().toStdString().c_str(), rosbag::bagmode::Read );

    rosbag::View bag_view ( bag, ros::TIME_MIN, ros::TIME_MAX, true );

    std::vector<const rosbag::ConnectionInfo *> connections = bag_view.getConnections();

    for(int i=0;  i<  connections.size(); i++)
    {
        all_topic_names.push_back( QString( connections[i]->topic.c_str() ) );
    }

    SelectXAxisDialog* dialog = new SelectXAxisDialog( &all_topic_names, false );
    dialog->exec();

    std::vector<int> topic_indexes = dialog->getSelectedRowNumber();

    std::set<std::string> topic_names;

    for (int i=0; i< topic_indexes.size(); i++)
    {
        topic_names.insert( all_topic_names.at( topic_indexes[i]).toStdString() );
    }

    std::map<std::string, TopicFlatContainer> topic_container;

    rosbag::View::iterator msg;
    for( msg = bag_view.begin(); msg!= bag_view.end(); msg++ )
    {
        if( topic_names.find( msg->getTopic()) == topic_names.end() ) {
            continue;
        }

        auto container = topic_container.find( msg->getTopic());

        if(  container == topic_container.end() )
        {
            topic_container.insert( std::make_pair(
                        msg->getTopic(),
                        buildFlatContainer( *msg, 64 ) ) );
        }
        else{
           //container->
        }

        //  ros::serialization::OStream stream( buffer, sizeof(buffer) );
        //   m->write( stream );

        qDebug() << msg->getTopic().c_str();
    }

    return plot_data;
}



DataLoadROS::~DataLoadROS()
{

}
