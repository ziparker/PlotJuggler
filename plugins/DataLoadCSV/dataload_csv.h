#ifndef DATALOAD_CSV_H
#define DATALOAD_CSV_H

#include <QObject>
#include <QtPlugin>
#include "../dataloader_base.h"


class  DataLoadCSV: public QObject, DataLoader
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.icarustechnology.Superplotter.DataLoader" "../dataloader.json")
    Q_INTERFACES(DataLoader)

public:
    DataLoadCSV();
    virtual const std::vector<const char*>& compatibleFileExtensions() const ;
    virtual PlotDataMap readDataFromFile(QFile* file, std::function<void(int)> updateCompletion, std::function<bool()> checkInterruption);

    virtual ~DataLoadCSV();

protected:
    int parseHeader(QFile *file,
                     std::vector<std::pair<bool, QString> > &ordered_names,
                     std::function<void(int)> updateCompletion);

private:
    std::vector<const char*> _extensions;


};

#endif // DATALOAD_CSV_H
