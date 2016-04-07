#ifndef DATALOAD_TEMPLATE_H
#define DATALOAD_TEMPLATE_H

#include <functional>
#include "plotdata.h"


class StatePublisher{

public:
    virtual void updateState(PlotDataMap* datamap, double current_time) = 0;
    virtual ~StatePublisher() {}
};

QT_BEGIN_NAMESPACE

#define StatePublisher_iid "com.icarustechnology.Superplotter.StatePublisher"

Q_DECLARE_INTERFACE(StatePublisher, StatePublisher_iid)

QT_END_NAMESPACE


#endif

