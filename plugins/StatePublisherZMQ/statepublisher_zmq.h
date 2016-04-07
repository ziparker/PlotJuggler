#ifndef DATALOAD_CSV_H
#define DATALOAD_CSV_H

#include <QObject>
#include <QtPlugin>
#include "../statepublisher_base.h"


class  StatePublisherZMQ: public QObject, StatePublisher
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.icarustechnology.Superplotter.StatePublisher" "../statepublisher.json")
    Q_INTERFACES(StatePublisher)

public:
    StatePublisherZMQ();

    virtual void updateState(PlotDataMap* datamap, double current_time);

    virtual ~StatePublisherZMQ();

private:
    PlotDataMap* _prev_dataplot;

    std::map<QString, double> _current_data;
    double _prev_time;

};

#endif // DATALOAD_CSV_H
