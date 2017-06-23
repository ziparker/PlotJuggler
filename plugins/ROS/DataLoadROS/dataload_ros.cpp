#include "dataload_ros.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QDebug>
#include <QApplication>
#include <QProgressDialog>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QProcess>
#include <rosbag/view.h>
#include <sys/sysinfo.h>

#include "../dialog_select_ros_topics.h"
#include "../shape_shifter_factory.hpp"
#include "../rule_editing.h"


DataLoadROS::DataLoadROS()
{
    _extensions.push_back( "bag");
}

const std::vector<const char*> &DataLoadROS::compatibleFileExtensions() const
{
    return _extensions;
}

size_t getAvailableRAM()
{
    struct sysinfo info;
    sysinfo(&info);
    return info.freeram;
}

PlotDataMap DataLoadROS::readDataFromFile(const QString &file_name,
                                          QString &load_configuration  )
{
    using namespace RosIntrospection;

    std::vector<std::pair<QString,QString>> all_topics;
    PlotDataMap plot_map;

    QFileInfo file_info(file_name);
    bool disable_caching = false;

    if(  file_info.size() > getAvailableRAM())
    {
        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(0, tr("Carefull"),
                                      QString("Your system has %1 MB for free memory (RAM)\n"
                                              "You are trying to load file that is %2 MB.\n\n"
                                              "If you load all the topics, you might experience problems.\n\n"
                                              "Do you want to DISABLE caching for messages which exceed 100 KB?\n"
                                              "Typically this includes images, maps, pointclouds, etc.\n\n"
                                              "If you select [YES], those messages cannot be republished.")
                                      .arg(getAvailableRAM() / (1024*1024))
                                      .arg(file_info.size() / (1024*1024)) ,
                                      QMessageBox::Yes|QMessageBox::No);

        if (reply == QMessageBox::Yes) {
            disable_caching = true;
        }
    }

    rosbag::Bag bag;
    try{
        bag.open( file_name.toStdString(), rosbag::bagmode::Read );
    }
    catch( rosbag::BagException&  ex)
    {
        QMessageBox::warning(0, tr("Error"),
                             QString("rosbag::open thrown an exception:\n")+
                             QString(ex.what()) );
        return plot_map;
    }

    rosbag::View bag_view ( bag, ros::TIME_MIN, ros::TIME_MAX, true );
    std::vector<const rosbag::ConnectionInfo *> connections = bag_view.getConnections();

    for(unsigned i=0;  i<connections.size(); i++)
    {
        const auto&  topic      =  connections[i]->topic;
        const auto&  md5sum     =  connections[i]->md5sum;
        const auto&  data_type  =  connections[i]->datatype;
        const auto&  definition =  connections[i]->msg_def;

        all_topics.push_back( std::make_pair(QString( topic.c_str()), QString( data_type.c_str()) ) );
        RosIntrospectionFactory::get().registerMessage(topic, md5sum, data_type, definition);
    }

    int count = 0;

    //----------------------------------
    QStringList default_topic_names = load_configuration.split(' ',QString::SkipEmptyParts);
    load_configuration.clear();

    DialogSelectRosTopics* dialog = new DialogSelectRosTopics( all_topics, default_topic_names );

    std::set<QString> topic_selected;

    if( dialog->exec() == static_cast<int>(QDialog::Accepted) )
    {
        const auto& selected_items = dialog->getSelectedItems();
        for(const auto& item: selected_items)
        {
            QString topic = item;
            topic_selected.insert( topic );

            // change the names in load_configuration
            load_configuration.append( topic ).append(" ");
        }
        // load the rules
        if( dialog->checkBoxUseRenamingRules()->isChecked())
        {
            _rules = RuleEditing::getRenamingRules();
        }
    }

    //-----------------------------------
    QProgressDialog progress_dialog;
    progress_dialog.setLabelText("Loading... please wait");
    progress_dialog.setWindowModality( Qt::ApplicationModal );

    rosbag::View bag_view_selected ( true );
    bag_view_selected.addQuery(bag, [&topic_selected](rosbag::ConnectionInfo const* connection)
    {
        return topic_selected.count( QString( connection->topic.c_str()) ) > 0;
    } );
    progress_dialog.setRange(0, bag_view_selected.size()-1);

    progress_dialog.show();

    QElapsedTimer timer;
    timer.start();

    ROSTypeFlat flat_container;

    for(const rosbag::MessageInstance& msg: bag_view_selected )
    {
        const std::string& topic  = msg.getTopic();

        SString topicname_SS;
        // WORKAROUND. There are some problems related to renaming when the character / is
        // used as prefix. We will remove that here.
        if( topic.at(0) == '/' )
            topicname_SS = SString( topic.data() +1,  topic.size()-1 );
        else
            topicname_SS = SString( topic.data(),  topic.size() );

        const auto& datatype   = msg.getDataType();
        auto msg_size = msg.size();

        std::vector<uint8_t> buffer ( msg_size );

        if( count++ %1000 == 0)
        {
            progress_dialog.setValue( count );
            QApplication::processEvents();

            if( progress_dialog.wasCanceled() ) {
                return PlotDataMap();
            }
        }

        ros::serialization::OStream stream(buffer.data(), buffer.size());

        // this single line takes almost the entire time of the loop
        msg.write(stream);

        auto typelist = RosIntrospectionFactory::get().getRosTypeList( topic );
        if( !typelist )
        {
            throw std::runtime_error("Can't retrieve the ROSTypeList from RosIntrospectionFactory");
        }
        buildRosFlatType( *typelist, datatype, topicname_SS, buffer.data(), &flat_container);
        applyNameTransform( _rules[datatype], &flat_container );

        // apply time offsets
        double msg_time;

        if(dialog->checkBoxUseHeaderStamp()->isChecked() == false)
        {
            msg_time = msg.getTime().toSec();
        }
        else{
            auto offset = FlatContainedContainHeaderStamp(flat_container);
            if(offset){
                msg_time = offset.value();
            }
            else{
                msg_time = msg.getTime().toSec();
            }
        }

        for(const auto& it: flat_container.renamed_value )
        {
            std::string field_name( it.first.data(), it.first.size());

            auto plot_pair = plot_map.numeric.find( field_name );
            if( plot_pair == plot_map.numeric.end() )
            {
                PlotDataPtr temp(new PlotData(field_name.c_str()));
                auto res = plot_map.numeric.insert( std::make_pair(field_name, temp ) );
                plot_pair = res.first;
            }

            PlotDataPtr& plot_data = plot_pair->second;
            plot_data->pushBack( PlotData::Point(msg_time, (double)it.second));
        } //end of for flat_container.renamed_value

        //-----------------------------------------
        // adding raw serialized topic for future uses.
        if( !disable_caching &&  msg_size > 100*1024*1024)
        {
            auto plot_pair = plot_map.user_defined.find( topic );

            if( plot_pair == plot_map.user_defined.end() )
            {
                PlotDataAnyPtr temp(new PlotDataAny(topic.c_str()));
                auto res = plot_map.user_defined.insert( std::make_pair( topic, temp ) );
                plot_pair = res.first;
            }
            PlotDataAnyPtr& plot_raw = plot_pair->second;
            plot_raw->pushBack( PlotDataAny::Point(msg_time, nonstd::any(std::move(buffer)) ));
        }
    }

    qDebug() << "The loading operation took" << timer.elapsed() << "milliseconds";
    return plot_map;
}


DataLoadROS::~DataLoadROS()
{

}


