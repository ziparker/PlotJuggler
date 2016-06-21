#ifndef PLOTDATA_RAW_H
#define PLOTDATA_RAW_H

#include <vector>
#include <memory>
#include <string>
#include <map>
#include <mutex>
#include <boost/circular_buffer.hpp>
#include <QDebug>

class PlotData
{
public:

    typedef struct{
        double min;
        double max;
    }Range;

    enum{
        MAX_CAPACITY = 1024*1024
    };

    PlotData();

    virtual ~PlotData() {}

    void setName(const std::string& name) { _name = name; }
    std::string name() const { return _name; }

    virtual size_t size();

    virtual size_t getIndexFromX(double x) const;

    double getYfromX(double x );

    std::pair<double,double> at(size_t index);

    void setCapacity(size_t capacity);

    void pushBack(double x, double y);

    void getColorHint(int *red, int* green, int* blue) const;

    void setColorHint(int red, int green, int blue);

    double setMaximumRangeX(double max_range);

    Range getRangeX();

    Range getRangeY();

protected:

    std::string _name;
    std::shared_ptr< boost::circular_buffer_space_optimized<double> >_x_points;
    std::shared_ptr< boost::circular_buffer_space_optimized<double> >_y_points;

    int _color_hint_red;
    int _color_hint_green;
    int _color_hint_blue;

private:
    bool _update_bounding_rect;

    double _max_range_X;
    std::mutex _mutex;

};

typedef std::shared_ptr<PlotData> PlotDataPtr;
typedef std::map<std::string, PlotDataPtr> PlotDataMap;


//-----------------------------------


inline PlotData::PlotData():
    _x_points( new boost::circular_buffer_space_optimized<double>() ),
    _y_points( new boost::circular_buffer_space_optimized<double>() ),
    _update_bounding_rect(true),
    _max_range_X( std::numeric_limits<double>::max() )
{
    setCapacity( MAX_CAPACITY );
}

inline void PlotData::setCapacity(size_t capacity)
{
    if( capacity > MAX_CAPACITY)
        capacity = MAX_CAPACITY;

    if( capacity < 2)
        capacity = 2;

    _x_points->set_capacity( capacity );
    _y_points->set_capacity( capacity );
}


inline void PlotData::pushBack(double x, double y)
{
    std::lock_guard<std::mutex> lock(_mutex);
    auto& X = _x_points;
    auto& Y = _y_points;

    size_t sizeX = X->size();

    if( sizeX > 2 && _max_range_X!= std::numeric_limits<double>::max())
    {
        double rangeX = X->back() - X->front();
        double delta = rangeX / (double)(sizeX - 1);
        size_t new_capacity = (size_t)( _max_range_X / delta);

        if( abs( new_capacity - sizeX) > 2 )
        {
            while( _x_points->size() > new_capacity)
            {
                _x_points->pop_front();
                _y_points->pop_front();
            }
           // qDebug() << "new capcaity " << new_capacity;
            setCapacity( new_capacity );
        }
    }

  //  qDebug() << "push " << x;
    X->push_back( x );
    Y->push_back( y );
}

inline size_t PlotData::getIndexFromX(double x ) const
{
    auto lower   = std::lower_bound(_x_points->begin(), _x_points->end(), x );
    size_t index = std::distance( _x_points->begin(), lower);
    return index;
}


inline double PlotData::getYfromX(double x)
{
    std::lock_guard<std::mutex> lock(_mutex);
    unsigned index = getIndexFromX(x);

    if( index >=0 && index < size())
    {
        return _y_points->at(index);
    }
    return 0;
}

inline std::pair<double,double> PlotData::at(size_t index)
{
 //   std::lock_guard<std::mutex> lock(_mutex);
    return std::make_pair( _x_points->at(index),  _y_points->at(index) );
}


inline size_t PlotData::size()
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _x_points->size();
}

inline void PlotData::getColorHint(int *red, int* green, int* blue) const
{
    *red = _color_hint_red;
    *green = _color_hint_green;
    *blue = _color_hint_blue;
}
inline void PlotData::setColorHint(int red, int green, int blue)
{
    _color_hint_red = red;
    _color_hint_green = green;
    _color_hint_blue = blue;
}


inline double PlotData::setMaximumRangeX(double max_range)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _max_range_X = max_range;
}

inline PlotData::Range PlotData::getRangeX()
{
    std::lock_guard<std::mutex> lock(_mutex);
    return  { _x_points->front(), _x_points->back() };
}

inline PlotData::Range PlotData::getRangeY()
{
    std::lock_guard<std::mutex> lock(_mutex);
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::min();

    for (size_t i=0; i< _y_points->size(); i++)
    {
        double Y = _y_points->at(i);
        if( Y < y_min ) y_min = Y;
        if( Y > y_max ) y_max = Y;
    }
    return  { y_min, y_max };
}



#endif // PLOTDATA_H
