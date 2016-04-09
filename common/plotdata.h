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
    std::string name() { return _name; }

    void setSubsampleFactor(int factor);

    size_t size() const;

    void setRangeX(double t_left, double t_right);

    int getIndexFromX(double x) const;

    double getY(double x ) const;

    SharedVector getVectorX();
    SharedVector getVectorY();

protected:

    std::string _name;

    SharedVector _x_points;
    SharedVector _y_points;

    int _subsample;

    float y_min, y_max;
    float x_min, x_max;
    int _index_first, _index_last;


};

typedef std::shared_ptr<PlotData> PlotDataPtr;
typedef std::map<std::string, PlotDataPtr> PlotDataMap;

#endif // PLOTDATA_H
