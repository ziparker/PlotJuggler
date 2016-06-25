#include "dataload_ros.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include "selectlistdialog.h"
#include <QDebug>

#include "rosbag/view.h"
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

PlotDataMap DataLoadROS::readDataFromFile(const std::string& file_name,
                                          std::function<void(int)> updateCompletion,
                                          std::function<bool()> checkInterruption,
                                          std::string &time_index_name  )
{

    using namespace RosTypeParser;

    QStringList all_topic_names;
    PlotDataMap plot_data;

    rosbag::Bag bag;
    bag.open( file_name, rosbag::bagmode::Read );

    rosbag::View bag_view ( bag, ros::TIME_MIN, ros::TIME_MAX, true );

    std::vector<const rosbag::ConnectionInfo *> connections = bag_view.getConnections();

    // create a list and a type map for each topic
    RosTypeMap type_map;

    for(int i=0;  i<  connections.size(); i++)
    {
        all_topic_names.push_back( QString( connections[i]->topic.c_str() ) );
        parseRosTypeDescription( connections[i]->datatype,
                                 connections[i]->msg_def,
                                 &type_map);
    }

    printRosTypeMap  (type_map );

    SelectFromListDialog* dialog = new SelectFromListDialog( &all_topic_names, false );
    dialog->exec();

    std::vector<int> topic_indexes = dialog->getSelectedRowNumber();

    std::set<std::string> topic_names;

    for (int i=0; i< topic_indexes.size(); i++)
    {
        topic_names.insert( all_topic_names.at( topic_indexes[i]).toStdString() );
    }

    std::vector<uint8_t> buffer ( 65*1024 );

    std::vector<SubstitutionRule> rules;
    rules.push_back( SubstitutionRule("/data.vectors[#].value",      "/data.vectors[#].name",   "#"));
    rules.push_back( SubstitutionRule("/data.doubles[#].value",      "/data.doubles[#].name",   "#"));
    rules.push_back( SubstitutionRule("/data.vectors3d[#].value[0]", "/data.vectors3d[#].name", "#.x"));
    rules.push_back( SubstitutionRule("/data.vectors3d[#].value[1]", "/data.vectors3d[#].name", "#.y"));
    rules.push_back( SubstitutionRule("/data.vectors3d[#].value[2]", "/data.vectors3d[#].name", "#.z"));

    int count = 0;

    for(rosbag::MessageInstance msg: bag_view )
    {
        double msg_time = msg.getTime().toSec();

        if( count++ %100 == 0)
        {
          //  qDebug() << count << " / " << bag_view.size() ;
            updateCompletion( 100*count / bag_view.size() );

            if( checkInterruption() == true ) return PlotDataMap();
        }

        RosTypeFlat flat_container;

        ros::serialization::OStream stream(buffer.data(), buffer.size());
        msg.write(stream);

        uint8_t* buffer_ptr = buffer.data();

        String datatype( msg.getDataType().data(), msg.getDataType().size() );
        String topic_name( msg.getTopic().data(), msg.getTopic().size() );

        buildRosFlatType(type_map, datatype, topic_name, &buffer_ptr,  &flat_container);
        applyNameTransform( rules, &flat_container );

        for(auto& it: flat_container.value_renamed )
        {
            std::string field_name ( it.first.data(), it.first.size());
            auto value = it.second;

            auto plot = plot_data.numeric.find( field_name );
            if( plot == plot_data.numeric.end() )
            {
                PlotDataPtr temp(new PlotData());
                temp->setCapacity( bag_view.size() );
                auto res = plot_data.numeric.insert( std::make_pair(field_name, temp ) );
                plot = res.first;
            }

            plot->second->pushBack( PlotData::Point(msg_time, value));
        }

     //   qDebug() << msg.getTopic().c_str();
    }

    return plot_data;
}



DataLoadROS::~DataLoadROS()
{

}
