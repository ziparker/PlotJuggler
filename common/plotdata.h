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
    typedef struct {
        int red;
        int green;
        int blue;
    }Color;


    PlotData();
    PlotData(SharedVector x, SharedVector y, std::string name);
    virtual ~PlotData() {}

    void addData( SharedVector x, SharedVector y );

    void setName(const std::string& name) { _name = name; }
    std::string name() const { return _name; }

    virtual size_t size() const;

    virtual int getIndexFromX(double x) const;

    void getColorHint(Color* color) const;
    void setColorHint(Color color);
    void setColorHint(int red, int green, int blue);

    double getY(double x ) const;

    SharedVector getVectorX() const;
    SharedVector getVectorY() const;

protected:

    std::string _name;
    Color _color_hint;

    SharedVector _x_points;
    SharedVector _y_points;

    double _y_min, _y_max;
    double _x_min, _x_max;

};

typedef std::shared_ptr<PlotData> PlotDataPtr;
typedef std::map<std::string, PlotDataPtr> PlotDataMap;

#endif // PLOTDATA_H
