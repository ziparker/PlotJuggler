#include "timeseries_qwt.h"
#include <limits>
#include <stdexcept>
#include <QMessageBox>
#include <QPushButton>
#include <QString>

TimeseriesQwt::TimeseriesQwt(const PlotData *base, double time_offset):
    DataSeriesBase(time_offset),
    _plot_data(base)
{
}

PlotData::RangeValueOpt TimeseriesQwt::getVisualizationRangeY(PlotData::RangeTime range_X)
{
    int first_index = getIndexFromTime(range_X.min);
    int last_index  = getIndexFromTime(range_X.max);

    if( first_index > last_index || first_index <0 || last_index <0 )
    {
        return PlotData::RangeValueOpt();
    }


    if( first_index == 0 && last_index == _cached_curve.size()-1)
    {
        return PlotData::RangeValueOpt( { _bounding_box.bottom(),
                                          _bounding_box.top() } );
    }

    double min_y =( std::numeric_limits<double>::max() );
    double max_y =(-std::numeric_limits<double>::max() );

    for (size_t i = first_index; i < last_index; i++)
    {
        const double Y = sample(i).y();
        min_y = std::min( min_y, Y );
        max_y = std::max( max_y, Y );
    }
    return PlotData::RangeValueOpt( { min_y, max_y } );
}


nonstd::optional<QPointF> TimeseriesQwt::sampleFromTime(double t)
{
    int index = getIndexFromTime(t);
    if( index < 0 )
    {
        return nonstd::optional<QPointF>();
    }
    const auto& p =_cached_curve.at( size_t(index) );
    return QPointF(p.x, p.y);
}

void TimeseriesQwt::onTimeoffsetChanged(double new_offset)
{
    double delta = new_offset - _time_offset;
    _time_offset = new_offset;
    for (auto& p: _cached_curve )
    {
        p.x -= delta;
    }
    _bounding_box.setLeft(  _bounding_box.left()  - delta );
    _bounding_box.setRight( _bounding_box.right() - delta );
}

int TimeseriesQwt::getIndexFromTime(double t)
{
    return _cached_curve.getIndexFromX( t - _time_offset );
}


bool Timeseries_NoTransform::updateCache()
{
    // No Transform
    if(_plot_data->size() == 0)
    {
        _cached_curve.clear();
        _bounding_box = QRectF();
        return true;
    }

    const size_t data_size = _plot_data->size();

    double min_y =( std::numeric_limits<double>::max() );
    double max_y =(-std::numeric_limits<double>::max() );

    _cached_curve.resize( data_size );

    for (size_t i=0; i < data_size; i++ )
    {
        const auto& p = _plot_data->at( i );
        min_y = std::min( min_y, p.y );
        max_y = std::max( max_y, p.y );
        _cached_curve[i] = { p.x - _time_offset, p.y };
    }

    _bounding_box.setLeft(  _cached_curve.front().x );
    _bounding_box.setRight( _cached_curve.back().x );
    _bounding_box.setBottom( min_y );
    _bounding_box.setTop( max_y );
    return true;
}

bool Timeseries_1stDerivative::updateCache()
{
    size_t data_size = _plot_data->size();

    if( data_size <= 1)
    {
        _cached_curve.clear();
        _bounding_box = QRectF();
        return true;
    }

    data_size = data_size - 1;
    _cached_curve.resize( data_size );

    double min_y =( std::numeric_limits<double>::max() );
    double max_y =(-std::numeric_limits<double>::max() );

    for (size_t i=0; i < data_size; i++ )
    {
        const auto& p0 = _plot_data->at( i );
        const auto& p1 = _plot_data->at( i+1 );
        const auto delta = p1.x - p0.x;
        const auto vel = (p1.y - p0.y) /delta;
        QPointF p( (p1.x + p0.x)*0.5, vel);
        p.setX( p.x() - _time_offset);
        _cached_curve[i] = { p.x(), p.y() };

        min_y = std::min( min_y, p.y() );
        max_y = std::max( max_y, p.y() );
    }

    _bounding_box.setLeft(  _cached_curve.front().x );
    _bounding_box.setRight( _cached_curve.back().x );
    _bounding_box.setBottom( min_y );
    _bounding_box.setTop( max_y );
    return true;
}

bool Timeseries_2ndDerivative::updateCache()
{
    size_t data_size = _plot_data->size();

    if( data_size <= 2)
    {
        _cached_curve.clear();
        _bounding_box = QRectF();
        return true;
    }

    data_size = data_size - 2;
    _cached_curve.resize( data_size );

    double min_y =( std::numeric_limits<double>::max() );
    double max_y =(-std::numeric_limits<double>::max() );

    for (size_t i=0; i < data_size; i++ )
    {
        const auto& p0 = _plot_data->at( i );
        const auto& p1 = _plot_data->at( i+1 );
        const auto& p2 = _plot_data->at( i+2 );
        const auto delta = (p2.x - p0.x) *0.5;
        const auto acc = ( p2.y - 2.0* p1.y + p0.y)/(delta*delta);
        QPointF p( (p2.x + p0.x)*0.5, acc );
        p.setX( p.x() - _time_offset);
        _cached_curve[i] = { p.x(), p.y() };

        min_y = std::min( min_y, p.y() );
        max_y = std::max( max_y, p.y() );
    }

    _bounding_box.setLeft(  _cached_curve.front().x );
    _bounding_box.setRight( _cached_curve.back().x );
    _bounding_box.setBottom( min_y );
    _bounding_box.setTop( max_y );
    return true;
}
