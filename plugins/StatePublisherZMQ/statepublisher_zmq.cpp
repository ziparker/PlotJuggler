#include "statepublisher_zmq.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>

StatePublisherZMQ::StatePublisherZMQ():
    _prev_dataplot(0),
    _prev_time( 0 )
{

}

StatePublisherZMQ::~StatePublisherZMQ()
{

}

void StatePublisherZMQ::updateState(PlotDataMap *datamap, double current_time)
{
    if( datamap == 0)
    {
        _prev_dataplot = datamap;
        _prev_time = current_time;
        _current_data.clear();
        return;
    }

    PlotDataMap::iterator it;
    if( datamap != _prev_dataplot || current_time != _prev_time)
    {
        for ( it = datamap->begin(); it != datamap->end(); it++ )
        {
            const QString& name = it->first;
            PlotDataPtr plotdata = it->second;
            _current_data[ name ] = plotdata->getY( current_time );
        }
    }
    _prev_dataplot = datamap;
    _prev_time = current_time;
}
