#ifndef DATALOAD_ROS_H
#define DATALOAD_ROS_H

#include <QObject>
#include <QtPlugin>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include "PlotJuggler/dataloader_base.h"
#include "../RosMsgParsers/ros_parser.h"

class  DataLoadROS: public DataLoader
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.icarustechnology.PlotJuggler.DataLoader" "../dataloader.json")
    Q_INTERFACES(DataLoader)

public:
    DataLoadROS();

    virtual const std::vector<const char*>& compatibleFileExtensions() const override;

    virtual PlotDataMapRef readDataFromFile(const QString& file_name, bool use_previous_configuration ) override;

    virtual const char* name() const override { return "DataLoad ROS bags"; }

    virtual ~DataLoadROS() override;

    virtual QDomElement xmlSaveState(QDomDocument &doc) const override;

    virtual bool xmlLoadState(QDomElement &parent_element ) override;

protected:
    void loadSubstitutionRule(QStringList all_topic_names);
    std::shared_ptr<rosbag::Bag> _bag;

private:
    RosMessageParser _ros_parser;

    std::vector<const char*> _extensions;

    QStringList _default_topic_names;

    bool _use_renaming_rules;

    std::vector<std::pair<QString, QString>> getAndRegisterAllTopics();

    void storeMessageInstancesAsUserDefined(PlotDataMapRef& plot_map,
                                            const std::string &prefix);
};

#endif // DATALOAD_CSV_H
