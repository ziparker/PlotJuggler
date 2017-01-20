#ifndef PLOTDATA_RAW_H
#define PLOTDATA_RAW_H

#include <vector>
#include <memory>
#include <string>
#include <map>
#include <mutex>
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>
#include <boost/any.hpp>
#include <QDebug>
#include <QColor>
#include <type_traits>

template <typename Time, typename Value> class PlotDataGeneric
{
public:

  struct RangeTime_{
    Time min;
    Time max;
  };

  struct RangeValue_{
    Value min;
    Value max;
  };

  typedef boost::optional<RangeTime_>  RangeTime;
  typedef boost::optional<RangeValue_> RangeValue;

  class Point{
  public:
    Time x;
    Value y;
    Point( Time _x, Value _y): x(_x), y(_y) {}
  };

  enum{
    MAX_CAPACITY = 1024*1024,
    MIN_CAPACITY = 10,
    DEFAULT_CAPACITY = 1024,
    ASYNC_BUFFER_CAPACITY = 1024
  };

  typedef Time    TimeType;

  typedef Value   ValueType;

  PlotDataGeneric();

  virtual ~PlotDataGeneric() {}

  void setName(const std::string& name) { _name = name; }
  std::string name() const { return _name; }

  virtual size_t size();

  int getIndexFromX(Time x);

  boost::optional<const Value &> getYfromX(Time x );

  Point at(size_t index);

  void setCapacity(size_t capacity);

  size_t capacity() const { return _capacity; }

  void pushBack(Point p);

  void pushBackAsynchronously(Point p);

  bool flushAsyncBuffer();

  QColor getColorHint() const;

  void setColorHint(QColor color);

  void setMaximumRangeX(Time max_range);


protected:

  std::string _name;
  boost::circular_buffer<Time>  _x_points;
  boost::circular_buffer<Value> _y_points;

  boost::circular_buffer<Point> _pushed_points;

  QColor _color_hint;

private:

  void updateCapacityBasedOnMaxTime();

  bool _update_bounding_rect;

  Time _max_range_X;
  std::mutex _mutex;
  size_t _capacity;
};


typedef PlotDataGeneric<double,double>  PlotData;
typedef PlotDataGeneric<double, boost::any> PlotDataAny;


typedef std::shared_ptr<PlotData>     PlotDataPtr;
typedef std::shared_ptr<PlotDataAny>  PlotDataAnyPtr;

typedef struct{
  std::map<std::string, PlotDataPtr>     numeric;
  std::map<std::string, PlotDataAnyPtr>  user_defined;
} PlotDataMap;


//-----------------------------------



template < typename Time, typename Value>
inline PlotDataGeneric <Time, Value>::PlotDataGeneric():
  _x_points( DEFAULT_CAPACITY ),
  _y_points( DEFAULT_CAPACITY ),
  _pushed_points( ASYNC_BUFFER_CAPACITY ),
  _update_bounding_rect(true),
  _max_range_X( std::numeric_limits<Time>::max() ),
  _capacity(1024 )
{
  static_assert( std::is_arithmetic<Time>::value ,"Only numbers can be used as time");
}

template < typename Time, typename Value>
inline void PlotDataGeneric<Time, Value>::setCapacity(size_t capacity)
{
  //std::lock_guard<std::mutex> lock(_mutex);
  if( capacity > MAX_CAPACITY)
    capacity = MAX_CAPACITY;

  if( capacity < MIN_CAPACITY)
    capacity = MIN_CAPACITY;

  _capacity = capacity;
  _x_points.set_capacity( capacity );
  _y_points.set_capacity( capacity );
}

template < typename Time, typename Value>
inline void PlotDataGeneric<Time, Value>::pushBack(Point point)
{
  _x_points.push_back( point.x );
  _y_points.push_back( point.y );
}

template < typename Time, typename Value>
inline void PlotDataGeneric<Time, Value>::pushBackAsynchronously(Point point)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _pushed_points.push_back( point );
}

template < typename Time, typename Value>
inline bool PlotDataGeneric<Time, Value>::flushAsyncBuffer()
{
  std::lock_guard<std::mutex> lock(_mutex);

  if( _pushed_points.empty() ) return false;

  while( !_pushed_points.empty() )
  {
      updateCapacityBasedOnMaxTime();
      const Point& point = _pushed_points.front();
      _x_points.push_back( point.x );
      _y_points.push_back( point.y );
      _pushed_points.pop_front();
  }
  return true;
}

template < typename Time, typename Value>
inline void PlotDataGeneric<Time, Value>::updateCapacityBasedOnMaxTime()
{
  const long sizeX      = _x_points.size();
  const size_t capacity = _x_points.capacity();

  if( sizeX >= 2 &&
      capacity >= MIN_CAPACITY &&
      capacity <= MAX_CAPACITY &&
      _max_range_X != std::numeric_limits<Time>::max())
  {
    const Time rangeX = _x_points.back() - _x_points.front();
    const Time delta = rangeX / (Time)(sizeX - 1);
    size_t new_capacity = ( _max_range_X / delta);

    if( labs( new_capacity - capacity) > (capacity*2)/100 ) // apply changes only if new capacity is > 2%
    {
      while( _x_points.size() > new_capacity)
      {
        _x_points.pop_front();
        _y_points.pop_front();
      }

      if( new_capacity > MAX_CAPACITY) new_capacity = MAX_CAPACITY;
      if( new_capacity < MIN_CAPACITY) new_capacity = MIN_CAPACITY;

      _x_points.set_capacity( new_capacity );
      _y_points.set_capacity( new_capacity );
    }
  }
}

template < typename Time, typename Value>
inline int PlotDataGeneric<Time, Value>::getIndexFromX(Time x )
{
 // std::lock_guard<std::mutex> lock(_mutex);
  if( _x_points.size() == 0 ){
    return -1;
  }
  auto lower = std::lower_bound(_x_points.begin(), _x_points.end(), x );
  auto index = std::distance( _x_points.begin(), lower);

  if( index >= _x_points.size() || index <0 )
  {
    return -1;
  }
  return index;
}


template < typename Time, typename Value>
inline boost::optional<const Value &> PlotDataGeneric<Time, Value>::getYfromX(Time x)
{
  //std::lock_guard<std::mutex> lock(_mutex);

  auto lower = std::lower_bound(_x_points.begin(), _x_points.end(), x );
  auto index = std::distance( _x_points.begin(), lower);

  if( index >= _x_points.size() || index < 0 )
  {
    return boost::optional<const Value&>();
  }
  return _y_points.at(index);
}

template < typename Time, typename Value>
inline typename PlotDataGeneric<Time, Value>::Point
PlotDataGeneric<Time, Value>::at(size_t index)
{
 // std::lock_guard<std::mutex> lock(_mutex);
  try{
    return { _x_points[index],  _y_points[index]  };
  }
  catch(...)
  {
    return { _x_points.back(),  _y_points.back() };
  }
}//


template < typename Time, typename Value>
inline size_t PlotDataGeneric<Time, Value>::size()
{
  //std::lock_guard<std::mutex> lock(_mutex);
  return _x_points.size();
}

template < typename Time, typename Value>
inline QColor PlotDataGeneric<Time, Value>::getColorHint() const
{
  return _color_hint;
}

template < typename Time, typename Value>
inline void PlotDataGeneric<Time, Value>::setColorHint(QColor color)
{
  _color_hint = color;
}


template < typename Time, typename Value>
inline void PlotDataGeneric<Time, Value>::setMaximumRangeX(Time max_range)
{
  //std::lock_guard<std::mutex> lock(_mutex);
  _max_range_X = max_range;
}




#endif // PLOTDATA_H
