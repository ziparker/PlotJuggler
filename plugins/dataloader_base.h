#ifndef DATALOAD_TEMPLATE_H
#define DATALOAD_TEMPLATE_H

#include <functional>
#include "plotdata.h"

QT_BEGIN_NAMESPACE
class QFile;
QT_END_NAMESPACE


class DataLoader{

public:
    virtual const std::vector<const char*>& compatibleFileExtensions() const = 0;
    virtual PlotDataMap readDataFromFile(QFile* file,
                                      std::function<void(int)> updateCompletion,
                                      std::function<bool()> checkInterruption) = 0;

    virtual ~DataLoader() {}
};

QT_BEGIN_NAMESPACE

#define DataRead_iid "com.icarustechnology.Superplotter.DataLoader"

Q_DECLARE_INTERFACE(DataLoader, DataRead_iid)

QT_END_NAMESPACE


#endif

