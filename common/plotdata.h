#ifndef PLOTDATA_RAW_H
#define PLOTDATA_RAW_H

#include <vector>
#include <memory>
#include <string>
#include <map>
#include <boost/circular_buffer.hpp>


class PlotData
{
public:

    PlotData();

    virtual ~PlotData() {}


    void setName(const std::string& name) { _name = name; }
    std::string name() const { return _name; }

    virtual size_t size() const;

    virtual size_t getIndexFromX(double x) const;

    double getY(double x ) const;

    void setCapacity(size_t capacity);

    void pushBack(double x, double y);

    void getColorHint(int *red, int* green, int* blue) const;

    void setColorHint(int red, int green, int blue);

    double minX();
    double minY();

    double maxX();
    double maxT();

protected:

    std::string _name;

    std::shared_ptr< boost::circular_buffer<double> >_x_points;
    std::shared_ptr< boost::circular_buffer<double> >_y_points;

    int _color_hint_red;
    int _color_hint_green;
    int _color_hint_blue;

private:
    bool _update_bounding_rect;

};

typedef std::shared_ptr<PlotData> PlotDataPtr;
typedef std::map<std::string, PlotDataPtr> PlotDataMap;


//-----------------------------------


inline PlotData::PlotData():
    _x_points( new boost::circular_buffer<double>() ),
    _y_points( new boost::circular_buffer<double>() )
{

}

inline void PlotData::setCapacity(size_t capacity){
    _x_points->set_capacity( capacity );
    _y_points->set_capacity( capacity );
}


inline void PlotData::pushBack(double x, double y){
    _x_points->push_back( x );
    _y_points->push_back( y );

}

inline size_t PlotData::getIndexFromX(double x ) const
{
    auto lower   = std::lower_bound(_x_points->begin(), _x_points->end(), x );
    size_t index = std::distance( _x_points->begin(), lower);
    return index;
}


inline double PlotData::getY(double x) const
{
    unsigned index = getIndexFromX(x);

    if( index >=0 && index < size())
    {
        return _y_points->at(index);
    }
    return 0;
}


inline size_t PlotData::size() const{
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





#endif // PLOTDATA_H
