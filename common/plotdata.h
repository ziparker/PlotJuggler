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
    boost::circular_buffer<Time>  _x_points;
    boost::circular_buffer<Value> _y_points;

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



template < typename Time, typename Value>
inline PlotDataGeneric <Time, Value>::PlotDataGeneric():
    _update_bounding_rect(true),
    _max_range_X( std::numeric_limits<Time>::max() )
{

    static_assert( std::is_arithmetic<Time>::value ,"Only numbers can be used as time");
    setCapacity( 1024 );
}

template < typename Time, typename Value>
inline void PlotDataGeneric<Time, Value>::setCapacity(size_t capacity)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if( capacity > MAX_CAPACITY)
        capacity = MAX_CAPACITY;

    if( capacity < 2)
        capacity = 2;

    _x_points.set_capacity( capacity );
    _y_points.set_capacity( capacity );
}


template < typename Time, typename Value>
inline void PlotDataGeneric<Time, Value>::pushBack(Point point)
{
    std::lock_guard<std::mutex> lock(_mutex);

    size_t sizeX = _x_points.size();

    if( sizeX > 2 && _max_range_X != std::numeric_limits<Time>::max())
    {
        double rangeX = _x_points.back() - _x_points.front();
        double delta = rangeX / (double)(sizeX - 1);
        size_t new_capacity = (size_t)( _max_range_X / delta);

        if( abs( new_capacity - sizeX) > 2 )
        {
            while( _x_points.size() > new_capacity)
            {
                _x_points.pop_front();
                _y_points.pop_front();
            }
            _x_points.set_capacity( new_capacity );
            _y_points.set_capacity( new_capacity );
        }
    }

    //  qDebug() << "push " << x;
    _x_points.push_back( point.x );
    _y_points.push_back( point.y );
}

template < typename Time, typename Value>
inline size_t PlotDataGeneric<Time, Value>::getIndexFromX(Time x )
{
    std::lock_guard<std::mutex> lock(_mutex);
    auto lower   = std::lower_bound(_x_points.begin(), _x_points.end(), x );
    size_t index = std::distance( _x_points.begin(), lower);

    if( index <0 || index >= size())
    {
        return -1;
    }
    return index;
}


template < typename Time, typename Value>
inline boost::optional<Value> PlotDataGeneric<Time, Value>::getYfromX(Time x)
{
    std::lock_guard<std::mutex> lock(_mutex);

    auto lower   = std::lower_bound(_x_points.begin(), _x_points.end(), x );
    size_t index = std::distance( _x_points.begin(), lower);

    if( index >=0 && index < size())
    {
        return _y_points.at(index);
    }
    return boost::optional<double>();
}

template < typename Time, typename Value>
inline typename PlotDataGeneric<Time, Value>::Point
PlotDataGeneric<Time, Value>::at(size_t index)
{
    std::lock_guard<std::mutex> lock(_mutex);
    try{
        return { _x_points.at(index),  _y_points.at(index)  };
    }
    catch(...)
    {
        return { _x_points.back(),  _y_points.back() };
    }
}//


template < typename Time, typename Value>
inline size_t PlotDataGeneric<Time, Value>::size()
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _x_points.size();
}

template < typename Time, typename Value>
inline void PlotDataGeneric<Time, Value>::getColorHint(int *red, int* green, int* blue) const
{
    *red = _color_hint_red;
    *green = _color_hint_green;
    *blue = _color_hint_blue;
}

template < typename Time, typename Value>
inline void PlotDataGeneric<Time, Value>::setColorHint(int red, int green, int blue)
{
    _color_hint_red = red;
    _color_hint_green = green;
    _color_hint_blue = blue;
}


template < typename Time, typename Value>
inline void PlotDataGeneric<Time, Value>::setMaximumRangeX(Time max_range)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _max_range_X = max_range;
}

template < typename Time, typename Value>
inline typename PlotDataGeneric<Time, Value>::RangeTime
PlotDataGeneric<Time, Value>::getRangeX()
{
    std::lock_guard<std::mutex> lock(_mutex);
    if( _x_points.size() < 2 )
        return {0,0} ;
    else
        return  { _x_points.front(), _x_points.back() };
}

template < typename Time, typename Value>
inline typename PlotDataGeneric<Time, Value>::RangeValue PlotDataGeneric<Time, Value>::getRangeY()
{
    std::lock_guard<std::mutex> lock(_mutex);
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::min();

    for (size_t i=0; i< _y_points.size(); i++)
    {
        double Y = _y_points.at(i);
        if( Y < y_min ) y_min = Y;
        if( Y > y_max ) y_max = Y;
    }
    return  { y_min, y_max };
}




#endif // PLOTDATA_H
