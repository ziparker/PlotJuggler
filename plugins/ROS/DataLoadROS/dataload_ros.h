#ifndef DATALOAD_ROS_H
#define DATALOAD_ROS_H

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <QObject>
#include <QtPlugin>
#include "dataloader_base.h"
#include <ros_type_introspection/ros_introspection.hpp>

class  DataLoadROS: public QObject, DataLoader
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.icarustechnology.Superplotter.DataLoader" "../../dataloader.json")
    Q_INTERFACES(DataLoader)

public:
    DataLoadROS();
    virtual const std::vector<const char*>& compatibleFileExtensions() const ;

    virtual PlotDataMap readDataFromFile(const std::string& file_name,
                                          std::string &load_configuration  );

    virtual ~DataLoadROS();

protected:
    void loadSubstitutionRule(QStringList all_topic_names);

private:
    std::vector<RosIntrospection::SubstitutionRule>  _rules;

    std::vector<const char*> _extensions;


};

#endif // DATALOAD_CSV_H
