#ifndef DATA_STREAMER_TEMPLATE_H
#define DATA_STREAMER_TEMPLATE_H

#include <QtPlugin>
#include <mutex>
#include "plotdata.h"


class DataStreamer{

public:

    virtual PlotDataMap& getDataMap() = 0;

<<<<<<< HEAD
=======
    virtual std::mutex& mutex() = 0;

>>>>>>> b821eb0b426188e0e1635851b6eb4b29d82b171e
    virtual void enableStreaming(bool enable) = 0;

    virtual bool isStreamingEnabled() const = 0;

    virtual ~DataStreamer() {}

<<<<<<< HEAD
    virtual const char* name() = 0;

=======
>>>>>>> b821eb0b426188e0e1635851b6eb4b29d82b171e
};

QT_BEGIN_NAMESPACE

#define DataStream_iid "com.icarustechnology.Superplotter.DataStreamer"

Q_DECLARE_INTERFACE(DataStreamer, DataStream_iid)

QT_END_NAMESPACE


#endif

