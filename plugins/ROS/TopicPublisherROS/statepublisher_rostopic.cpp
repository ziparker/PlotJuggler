#include "statepublisher_rostopic.h"
#include "PlotJuggler/any.hpp"
#include "../shape_shifter_factory.hpp"
#include "../qnodedialog.h"
#include "ros_type_introspection/ros_introspection.hpp"
#include <QDialog>
#include <QFormLayout>
#include <QCheckBox>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QDialogButtonBox>
#include <QScrollArea>
#include <QPushButton>
#include <QSettings>
#include <rosbag/bag.h>
#include <std_msgs/Header.h>
#include <unordered_map>
#include <rosgraph_msgs/Clock.h>

TopicPublisherROS::TopicPublisherROS():
    enabled_(false ),
    _node(nullptr),
    _publish_clock(true)
{
    QSettings settings;
    _publish_clock = settings.value( "TopicPublisherROS/publish_clock", true ).toBool();

}

TopicPublisherROS::~TopicPublisherROS()
{
    enabled_ = false;
}

void TopicPublisherROS::setParentMenu(QMenu *menu)
{
    _menu = menu;

    _select_topics_to_publish = new QAction(QString("Select topics to be published"), _menu);
    _menu->addAction( _select_topics_to_publish );
    connect(_select_topics_to_publish, &QAction::triggered,
            this, &TopicPublisherROS::filterDialog);
}

void TopicPublisherROS::setEnabled(bool to_enable)
{  
    if( !_node && to_enable)
    {
        _node = RosManager::getNode();
    }
    enabled_ = (to_enable && _node);

    if(enabled_)
    {
        filterDialog();
        if( !_tf_publisher)
        {
            _tf_publisher = std::unique_ptr<tf::TransformBroadcaster>( new tf::TransformBroadcaster );
        }
        _previous_published_msg.clear();
    }
}

void TopicPublisherROS::filterDialog(bool)
{   
    auto all_topics = RosIntrospectionFactory::get().getTopicList();

    if( all_topics.empty() ) return;

    QDialog* dialog = new QDialog();
    dialog->setWindowTitle("Select topics to be published");
    dialog->setMinimumWidth(350);
    QVBoxLayout* vertical_layout = new QVBoxLayout();
    QFormLayout* grid_layout = new QFormLayout();

    std::map<std::string, QCheckBox*> checkbox;

    QFrame* frame = new QFrame;
    QCheckBox* publish_sim_time = new QCheckBox("Publish /clock");
    QPushButton* select_button = new QPushButton("Select all");
    QPushButton* deselect_button = new QPushButton("Deselect all");

    publish_sim_time->setChecked( _publish_clock );
    publish_sim_time->setFocusPolicy(Qt::NoFocus);
    select_button->setFocusPolicy(Qt::NoFocus);
    deselect_button->setFocusPolicy(Qt::NoFocus);

    for (const auto& topic: all_topics)
    {
        auto cb = new QCheckBox(dialog);
        auto filter_it = _topics_to_publish.find( *topic );
        if( filter_it == _topics_to_publish.end() )
        {
            cb->setChecked( true );
        }
        else{
            cb->setChecked( filter_it->second );
        }
        cb->setFocusPolicy(Qt::NoFocus);
        grid_layout->addRow( new QLabel( QString::fromStdString(*topic)), cb);
        checkbox.insert( std::make_pair(*topic, cb) );
        connect( select_button,   &QPushButton::pressed, [cb](){ cb->setChecked(true);} );
        connect( deselect_button, &QPushButton::pressed, [cb](){ cb->setChecked(false);} );
    }

    frame->setLayout(grid_layout);

    QScrollArea* scrollArea = new QScrollArea;
    scrollArea->setWidget(frame);

    QDialogButtonBox* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);

    QHBoxLayout* select_buttons_layout = new QHBoxLayout;
    select_buttons_layout->addWidget( select_button );
    select_buttons_layout->addWidget( deselect_button );

    vertical_layout->addWidget( publish_sim_time );
    vertical_layout->addWidget(scrollArea);
    vertical_layout->addLayout(select_buttons_layout);
    vertical_layout->addWidget( buttons );

    connect(buttons, SIGNAL(accepted()), dialog, SLOT(accept()));
    connect(buttons, SIGNAL(rejected()), dialog, SLOT(reject()));

    dialog->setLayout(vertical_layout);
    auto result = dialog->exec();

    if(result == QDialog::Accepted)
    {
        _topics_to_publish.clear();
        for(const auto& it: checkbox )
        {
            _topics_to_publish.insert( {it.first, it.second->isChecked() } );
        }

        //remove already created publisher if not needed anymore
        for (auto it = _publishers.begin(); it != _publishers.end(); /* no increment */)
        {
            const std::string& topic_name = it->first;
            if( !toPublish(topic_name) )
            {
                it = _publishers.erase(it);
            }
            else{
                it++;
            }
        }

        _publish_clock = publish_sim_time->isChecked();

        if( _publish_clock )
        {
            _clock_publisher = _node->advertise<rosgraph_msgs::Clock>( "/clock", 10, true);
        }
        else{
            _clock_publisher.shutdown();
        }

        QSettings settings;
        settings.setValue( "TopicPublisherROS/publish_clock", _publish_clock );
    }
}

void TopicPublisherROS::broadcastTF(double current_time)
{
    const ros::Time ros_time = ros::Time::now();

    std::unordered_map<std::string, geometry_msgs::TransformStamped> transforms;

    for(const auto& data_it:  _datamap->user_defined )
    {
        const std::string& topic_name = data_it.first;
        const PlotDataAny& plot_any = data_it.second;

        if( !toPublish(topic_name) )
        {
            continue;// Not selected
        }
        const RosIntrospection::ShapeShifter* shapeshifter =
                RosIntrospectionFactory::get().getShapeShifter( topic_name );
        if( shapeshifter->getDataType() != "tf/tfMessage" &&
            shapeshifter->getDataType() != "tf2_msgs/TFMessage"   )
        {
            continue;
        }

         const PlotDataAny* tf_data = &plot_any;
         int last_index = tf_data->getIndexFromX( current_time );
         if( last_index < 0)
         {
             continue;
         }

         std::vector<uint8_t> raw_buffer;
         // 1 second in the past (to be configurable in the future
         int initial_index = tf_data->getIndexFromX( current_time - 2.0 );

         for(size_t index = std::max(0, initial_index); index <= last_index; index++ )
         {
             const nonstd::any& any_value = tf_data->at(index).y;

             const bool isRosbagMessage = any_value.type() == typeid(rosbag::MessageInstance);

             if( isRosbagMessage )
             {
                 const auto& msg_instance = nonstd::any_cast<rosbag::MessageInstance>( any_value );
                 raw_buffer.resize( msg_instance.size() );
                 ros::serialization::OStream ostream(raw_buffer.data(), raw_buffer.size());
                 msg_instance.write(ostream);

                 tf::tfMessage tf_msg;
                 ros::serialization::IStream istream( raw_buffer.data(), raw_buffer.size() );
                 ros::serialization::deserialize(istream, tf_msg);

                 for(const auto& stamped_transform: tf_msg.transforms)
                 {
                     const auto& child_id = stamped_transform.child_frame_id;
                     auto it = transforms.find(child_id);
                     if( it == transforms.end())
                     {
                         transforms.insert( {stamped_transform.child_frame_id, stamped_transform} );
                     }
                     else if( it->second.header.stamp <= stamped_transform.header.stamp)
                     {
                         it->second = stamped_transform;
                     }
                 }
             }
         }
    }

    std::vector<geometry_msgs::TransformStamped> transforms_vector;
    transforms_vector.reserve(transforms.size());

    for(auto& trans: transforms)
    {
        trans.second.header.stamp = ros_time;
        transforms_vector.emplace_back( std::move(trans.second) );
    }

    _tf_publisher->sendTransform(transforms_vector);
}

bool TopicPublisherROS::toPublish(const std::string &topic_name)
{
    auto it = _topics_to_publish.find( topic_name );
    if( it == _topics_to_publish.end() ){

        return false;
    }
    else {
        return it->second;
    }
}



void TopicPublisherROS::updateState(double current_time)
{
    if(!enabled_ || !_node) return;

    const ros::Time ros_time = ros::Time::now();

    //-----------------------------------------------
    broadcastTF(current_time);
    //-----------------------------------------------

    int skipped = 0;
    int sent_count = 0;
    int filtered_out = 0;

    ros::Time msg_time;

    for(const auto& data_it:  _datamap->user_defined )
    {
        const std::string& topic_name = data_it.first;
        const PlotDataAny& plot_any = data_it.second;
        if( !toPublish(topic_name) )
        {
            filtered_out++;
            continue;// Not selected
        }
        const RosIntrospection::ShapeShifter* shapeshifted = RosIntrospectionFactory::get().getShapeShifter( topic_name );
        if( ! shapeshifted )
        {
            continue;// Not registered, just skip
        }
        if( shapeshifted->getDataType() == "tf/tfMessage" ||
            shapeshifted->getDataType() == "tf2_msgs/TFMessage"  )
        {
            continue;
        }

        RosIntrospection::ShapeShifter shapeshifted_msg = *shapeshifted;
        int last_index = plot_any.getIndexFromX( current_time );
        if( last_index < 0)
        {
            continue;
        }

        const auto& any_value = plot_any.at( last_index ).y;

        const bool isRawBuffer     = any_value.type() == typeid( std::vector<uint8_t>);
        const bool isRosbagMessage = any_value.type() == typeid(rosbag::MessageInstance);

        auto prev_it = _previous_published_msg.find( &plot_any );
        if( prev_it == _previous_published_msg.end() || prev_it->second != last_index)
        {
            _previous_published_msg.insert( { &plot_any, last_index } );
        }
        else{
            skipped++;
            continue;
        }
        sent_count++;

        std::vector<uint8_t> raw_buffer;

        if( isRawBuffer){
            raw_buffer = nonstd::any_cast<std::vector<uint8_t>>( any_value );
        }
        else if( isRosbagMessage ){
            const rosbag::MessageInstance& msg_instance = nonstd::any_cast<rosbag::MessageInstance>( any_value );
            raw_buffer.resize( msg_instance.size() );
            ros::serialization::OStream stream(raw_buffer.data(), raw_buffer.size());
            msg_instance.write(stream);
            msg_time = msg_instance.getTime();
        }
        else{
            continue;
        }


        ros::serialization::IStream stream( raw_buffer.data(), raw_buffer.size() );
        shapeshifted_msg.read( stream );

        auto publisher_it = _publishers.find( topic_name );
        if( publisher_it == _publishers.end())
        {
            auto res = _publishers.insert( {topic_name, shapeshifted_msg.advertise( *_node, topic_name, 10, true)} );
            publisher_it = res.first;
        }

        const ros::Publisher& publisher = publisher_it->second;
        publisher.publish( shapeshifted_msg );
    }

    if( _publish_clock )
    {
        rosgraph_msgs::Clock clock;
        clock.clock = msg_time;
       _clock_publisher.publish( clock );
    }

    //qDebug() << filtered_out << " + " << skipped << " + " << sent_count << " = " << _datamap->user_defined.size();
}
