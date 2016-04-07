#ifndef DATALOAD_CSV_H
#define DATALOAD_CSV_H

#include <QObject>
#include <QtPlugin>
#include "../dataloader_base.h"


class  DataLoadCSV: public QObject, DataLoader
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID DataRead_iid "../dataloader.json")
    Q_INTERFACES(DataLoader)

public:
    DataLoadCSV();
    virtual const std::vector<const char*>& compatibleFileExtensions() const ;
    virtual PlotDataMap readDataFromFile(QFile* file, std::function<void(int)> updateCompletion, std::function<bool()> checkInterruption);

    virtual ~DataLoadCSV();

private:
    std::vector<const char*> _extensions;


};

#endif // DATALOAD_CSV_H
