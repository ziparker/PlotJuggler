#ifndef PLOTDATA_RAW_H
#define PLOTDATA_RAW_H

#include <vector>
#include <memory>
#include <string>
#include <map>
#include <mutex>
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>
#include <QDebug>
#include <type_traits>

template <typename Time, typename Value> class PlotDataGeneric
{
public:

    typedef struct{
        Time min;
        Time max;
    }RangeTime;

    typedef struct{
        Value min;
        Value max;
    }RangeValue;

    class Point{
    public:
        Time x;
        Value y;
        Point( Time _x, Value _y): x(_x), y(_y) {}
    };

    enum{
        MAX_CAPACITY = 1024*1024
    };

    typedef Time    TimeType;

    typedef Value   ValueType;

    PlotDataGeneric();

    virtual ~PlotDataGeneric() {}

    void setName(const std::string& name) { _name = name; }
    std::string name() const { return _name; }

    virtual size_t size();

    size_t getIndexFromX(Time x);

    boost::optional<Value> getYfromX(Time x );

    Point at(size_t index);

    void setCapacity(size_t capacity);

    void pushBack(Point p);

    void getColorHint(int *red, int* green, int* blue) const;

    void setColorHint(int red, int green, int blue);

    void setMaximumRangeX(Time max_range);

    RangeTime getRangeX();

    RangeValue getRangeY();

protected:

    std::string _name;
    std::shared_ptr< boost::circular_buffer<Time> >_x_points;
    std::shared_ptr< boost::circular_buffer<Value> >_y_points;

    int _color_hint_red;
    int _color_hint_green;
    int _color_hint_blue;

private:
    bool _update_bounding_rect;

    Time _max_range_X;
    std::mutex _mutex;

};


typedef PlotDataGeneric<float,double>  PlotData;
typedef PlotDataGeneric<float,void*>   PlotDataVoid;


typedef std::shared_ptr<PlotData>     PlotDataPtr;
typedef std::shared_ptr<PlotDataVoid> PlotDataVoidPtr;

typedef struct{
    std::map<std::string, PlotDataPtr>     numeric;
    std::map<std::string, PlotDataVoidPtr> user_defined;
} PlotDataMap;


//-----------------------------------

#include "plotdata.impl.h"

#endif // PLOTDATA_H
