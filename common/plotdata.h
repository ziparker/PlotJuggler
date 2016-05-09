#ifndef PLOTDATA_RAW_H
#define PLOTDATA_RAW_H

#include <vector>
#include <memory>
#include <memory>
#include <string>
#include <map>

typedef std::shared_ptr<std::vector<double> > SharedVector;

class PlotData
{
public:

    PlotData();
    PlotData(SharedVector x, SharedVector y, std::string name);
    virtual ~PlotData() {}

    void addData( SharedVector x, SharedVector y );

    void setName(const std::string& name) { _name = name; }
    std::string name() const { return _name; }

    virtual size_t size() const;

    virtual int getIndexFromX(double x) const;

    double getY(double x ) const;

    SharedVector getVectorX() const;
    SharedVector getVectorY() const;

    void getColorHint(int *red, int* green, int* blue) const;
    void setColorHint(int red, int green, int blue);

protected:

    std::string _name;

    SharedVector _x_points;
    SharedVector _y_points;

    double _y_min, _y_max;
    double _x_min, _x_max;
    int _red, _green, _blue;

};

typedef std::shared_ptr<PlotData> PlotDataPtr;
typedef std::map<std::string, PlotDataPtr> PlotDataMap;


//-----------------------------------


inline PlotData::PlotData(){ }

inline PlotData::PlotData(SharedVector x, SharedVector y, std::string name)
{
    _name = name;
    addData(x,y);
}


inline void PlotData::addData(SharedVector x, SharedVector y)
{
    _x_points = x;
    _y_points = y;

    if( x->size() != y->size() ) {
        throw std::runtime_error("size of x and y vectors must match");
    }

    _y_min = std::numeric_limits<double>::max();
    _y_max = std::numeric_limits<double>::min();

    _x_min = _x_points->front();
    _x_max = _x_points->back();

    for (unsigned i=0; i< x->size(); i++) {
        double Y = _y_points->at(i);
        if( Y < _y_min ) _y_min = Y;
        if( Y > _y_max ) _y_max = Y;
    }
}


inline int PlotData::getIndexFromX(double x ) const
{
    static double prev_query = std::numeric_limits<double>::min();
    static int prev_index = 0;

    if( x == prev_query) {
        return prev_index;
    }
    prev_query = x;

    std::vector<double>::iterator lower;
    lower = std::lower_bound(_x_points->begin(), _x_points->end(), x );
    int index =   std::distance( _x_points->begin(), lower);

    prev_index = index;
    return index;
}


inline double PlotData::getY(double x) const
{
    unsigned index = getIndexFromX(x);
    if( index >0 && index < size()) {
        return _y_points->at(index);
    }
    return 0;
}

inline SharedVector PlotData::getVectorX() const {
    return _x_points;
}

inline SharedVector PlotData::getVectorY() const {
    return _y_points;
}

inline size_t PlotData::size() const{
   return _x_points->size();
}

inline void PlotData::getColorHint(int *red, int* green, int* blue) const
{
    *red = _red;
    *green = _green;
    *blue = _blue;
}
inline void PlotData::setColorHint(int red, int green, int blue)
{
    _red = red;
    _green = green;
    _blue = blue;
}



#endif // PLOTDATA_H
