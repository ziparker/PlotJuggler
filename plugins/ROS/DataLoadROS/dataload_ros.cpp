#include "dataload_ros.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QDebug>
#include <QApplication>
#include <QProgressDialog>
#include <QElapsedTimer>

#include <rosbag/view.h>

#include "dialog_select_ros_topics.h"
#include "../ruleloaderwidget.h"

DataLoadROS::DataLoadROS()
{
  _extensions.push_back( "bag");
}

const std::vector<const char*> &DataLoadROS::compatibleFileExtensions() const
{
  return _extensions;
}


PlotDataMap DataLoadROS::readDataFromFile(const std::string& file_name,
                                          std::string &load_configuration  )
{
  using namespace RosIntrospection;

  QStringList all_topic_names;
  PlotDataMap plot_map;

  rosbag::Bag bag;
  bag.open( file_name, rosbag::bagmode::Read );


  rosbag::View bag_view ( bag, ros::TIME_MIN, ros::TIME_MAX, true );
  auto first_time = bag_view.getBeginTime();
  std::vector<const rosbag::ConnectionInfo *> connections = bag_view.getConnections();

  // create a list and a type map for each topic
  std::map<std::string,ROSTypeList> type_map;

  for(unsigned i=0;  i<connections.size(); i++)
  {
    all_topic_names.push_back( QString( connections[i]->topic.c_str() ) );

    const auto&  data_type =  connections[i]->datatype;
    auto topic_map = buildROSTypeMapFromDefinition( data_type,
                                                    connections[i]->msg_def);
    type_map.insert( std::make_pair(data_type,topic_map));
  }

  all_topic_names.sort();

  int count = 0;

  DialogSelectRosTopics* dialog = new DialogSelectRosTopics( all_topic_names );

  std::set<std::string> topic_selected;

  std::vector<uint8_t> buffer ( 64*1024 );

  if( dialog->exec() == QDialog::Accepted)
  {
    const auto& selected_items = dialog->getSelectedItems();
    for(auto item: selected_items)
    {
      topic_selected.insert( item.toStdString() );
    }
    // load the rules
    _rules = dialog->getLoadedRules();

  }

  rosbag::View bag_view_reduced ( true );
  bag_view_reduced.addQuery(bag, [topic_selected](rosbag::ConnectionInfo const* connection)
  {
    return topic_selected.find( connection->topic ) != topic_selected.end();
  } );

  QProgressDialog progress_dialog;
  progress_dialog.setLabelText("Loading... please wait");
  progress_dialog.setWindowModality( Qt::ApplicationModal );
  progress_dialog.setRange(0, bag_view_reduced.size() -1);
  progress_dialog.show();

  QElapsedTimer timer;
  timer.start();


  ROSTypeFlat flat_container;

  for(const rosbag::MessageInstance& msg: bag_view_reduced )
  {
   /* if( topic_selected.find( msg.getTopic() ) == topic_selected.end() )
    {
      continue;
    }*/

    const auto& data_type = msg.getDataType();

    double msg_time = (msg.getTime() - first_time).toSec();

    if( count++ %1000 == 0)
    {
      qDebug() << count << " / " << bag_view_reduced.size();

      progress_dialog.setValue( count );
      QApplication::processEvents();

      if( progress_dialog.wasCanceled() ) {
        return PlotDataMap();
      }
    }

    ros::serialization::OStream stream(buffer.data(), buffer.size());

    // this single line takes almost the entire time of the loop
    msg.write(stream);

    uint8_t* buffer_ptr = buffer.data();

    ROSType datatype(  msg.getDataType() );
    SString topic_name( msg.getTopic().data(),  msg.getTopic().size() );

    buildRosFlatType(type_map[ data_type ], datatype, topic_name, &buffer_ptr, &flat_container);

    applyNameTransform( _rules[data_type], &flat_container );

    for(auto& it: flat_container.renamed_value )
    {
      std::string field_name ( it.first.data(), it.first.size());
      auto value = it.second;

      auto plot_pair = plot_map.numeric.find( field_name );
      if( plot_pair == plot_map.numeric.end() )
      {
        PlotDataPtr temp(new PlotData());
        //temp->setCapacity( bag_view_reduced.size() );
        auto res = plot_map.numeric.insert( std::make_pair(field_name, temp ) );
        plot_pair = res.first;
      }

      PlotDataPtr& plot_data = plot_pair->second;

      if( plot_data->size() >= plot_data->capacity() ) // auto increase size
      {
        plot_data->setCapacity( plot_data->size()*2 );
      }
      plot_data->pushBack( PlotData::Point(msg_time, value));
    }
    //   qDebug() << msg.getTopic().c_str();
  }
  qDebug() << "The loading operation took" << timer.elapsed() << "milliseconds";

  return plot_map;
}



DataLoadROS::~DataLoadROS()
{

}


