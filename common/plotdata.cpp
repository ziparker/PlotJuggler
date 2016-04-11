#include "plotdata.h"
#include <limits>
#include <algorithm>
#include <stdexcept>

PlotData::PlotData()
{

}

PlotData::PlotData(SharedVector x, SharedVector y, std::string name)
{
    _name = name;
    addData(x,y);
}


void PlotData::addData(SharedVector x, SharedVector y)
{
    _x_points = x;
    _y_points = y;

    if( x->size() != y->size() )
    {
        throw std::runtime_error("size of x and y vectors must match");
    }

    _y_min = std::numeric_limits<double>::max();
    _y_max = std::numeric_limits<double>::min();

    _x_min = _x_points->front();
    _x_max = _x_points->back();

    for (unsigned i=0; i< x->size(); i++)
    {
        double Y = _y_points->at(i);
        if( Y < _y_min ) _y_min = Y;
        if( Y > _y_max ) _y_max = Y;
    }
}



size_t PlotData::size() const
{
   return _x_points->size();
}



int PlotData::getIndexFromX(double x ) const
{
    static double prev_query = std::numeric_limits<double>::min();
    static int prev_index = 0;

    if( x == prev_query)
    {
        return prev_index;
    }
    prev_query = x;

    std::vector<double>::iterator lower;
    lower = std::lower_bound(_x_points->begin(), _x_points->end(), x );
    int index =   std::distance( _x_points->begin(), lower);

    prev_index = index;
    return index;
}

void PlotData::getColorHint(PlotData::Color *color) const
{
    *color = _color_hint;
}

void PlotData::setColorHint(PlotData::Color color)
{
    _color_hint = color;
}

void PlotData::setColorHint(int red, int green, int blue)
{
    _color_hint.red   = red;
    _color_hint.green = green;
    _color_hint.blue  = blue;
}

double PlotData::getY(double x) const
{
    unsigned index = getIndexFromX(x);
    if( index >0 && index < size())
    {
        return _y_points->at(index);
    }
    return 0;
}

SharedVector PlotData::getVectorX() const { return _x_points; }

SharedVector PlotData::getVectorY() const { return _y_points; }

