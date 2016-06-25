#ifndef DATALOAD_ROS_H
#define DATALOAD_ROS_H

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <QObject>
#include <QtPlugin>
#include "../dataloader_base.h"


class  DataLoadROS: public QObject, DataLoader
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.icarustechnology.Superplotter.DataLoader" "../dataloader.json")
    Q_INTERFACES(DataLoader)

public:
    DataLoadROS();
    virtual const std::vector<const char*>& compatibleFileExtensions() const ;

    virtual PlotDataMap readDataFromFile(const std::string& file_name,
                                          std::function<void(int)> updateCompletion,
                                          std::function<bool()> checkInterruption,
                                          std::string &time_index_name  );

    virtual ~DataLoadROS();

protected:
    int parseHeader(QFile *file,
                     std::vector<std::pair<bool, QString> > &ordered_names,
                     std::function<void(int)> updateCompletion);

private:
    std::vector<const char*> _extensions;


};

#endif // DATALOAD_CSV_H
