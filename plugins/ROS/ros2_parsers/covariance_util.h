#pragma once

#include "fmt/format.h"
#include "ros2_parser.h"

using namespace Ros2Introspection;

template <size_t N>
class CovarianceParser
{
public:
    CovarianceParser(const std::string& prefix)
    {
      int index = 0;
      for (int i = 0; i < N; i++) {
        for (int j = i; j < N; j++) {
          _key.push_back( fmt::format( "{}[{},{}]", prefix, i, j ) );
        }
      }
    }

    void parse(PlotDataMapRef &plot_data,
               const std::array<double,N*N>& covariance,
               double timestamp)
    {
      size_t s = 0;
      for (int i = 0; i < N; i++) {
        for (int j = i; j < N; j++) {
          auto& series = MessageParserBase::getSeries(plot_data, _key[s++] );
          series.pushBack({timestamp, covariance[i*N + j]});
        }
      }
    }

private:
    std::vector<std::string> _key;
};

