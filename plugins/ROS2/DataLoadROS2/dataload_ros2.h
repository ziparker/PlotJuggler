#ifndef DATALOAD_ROS2_H
#define DATALOAD_ROS2_H

#include <QObject>
#include <QtPlugin>
#include <QSettings>

#include <rosbag2/readers/sequential_reader.hpp>
#include "PlotJuggler/optional.hpp"
#include "PlotJuggler/dataloader_base.h"
#include "ros2_introspection/ros2_introspection.hpp"
#include "../dialog_select_ros_topics.h"


class  DataLoadROS2: public DataLoader
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.robopec.PlotJuggler.ROS2DataLoader" "../dataloader.json")
    Q_INTERFACES(DataLoader)

  public:
    DataLoadROS2();

    virtual const std::vector<const char*>& compatibleFileExtensions() const override;

    virtual bool readDataFromFile(FileLoadInfo* fileload_info, PlotDataMapRef& destination) override;

    virtual const char* name() const override { return "DataLoad ROS2 bags"; }

    virtual ~DataLoadROS2() override = default;

    virtual bool xmlSaveState(QDomDocument &doc, QDomElement &parent_element) const override;

    virtual bool xmlLoadState(const QDomElement &parent_element ) override;

  private:

    struct TopicInfo{
        bool has_header_stamp;
        std::shared_ptr<rosbag2_introspection_message_t> buffer;
        const rosidl_message_type_support_t *type_support;
        Ros2Introspection::FlatMessage flat_msg;
        Ros2Introspection::RenamedValues renamed;
    };

    std::unordered_map<std::string,TopicInfo> _topic_info;

    std::shared_ptr<rosbag2::readers::SequentialReader> _bagReader;

    Ros2Introspection::Parser _parser;

    std::vector<const char*> _extensions;

    DialogSelectRosTopics::Configuration _config;

    std::vector<std::pair<QString, QString>> getAndRegisterAllTopics();

    void saveDefaultSettings();

    void loadDefaultSettings();

};

#endif // DATALOAD_ROS2_H
