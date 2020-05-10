#pragma once

#include "fmt/format.h"
#include "ros2_parser.h"

using namespace Ros2Introspection;

inline void ParseCovariance(const std::string &prefix,
                            PlotDataMapRef &plot_data,
                            const std::array<double,9>& covariance,
                            double timestamp)
{
    static std::array<std::string,9> suffix = []()
    {
        std::array<std::string,9> out;
        int index = 0;
        for (int i = 0; i < 3; i++) {
            for (int j = i; j < 3; j++) {
                out[index++] = fmt::format( "[{},{}]", i, j );
            }
        }
        return out;
    }();

    size_t s = 0;
    for (int i = 0; i < 3; i++) {
        for (int j = i; j < 3; j++) {
            auto& series = MessageParserBase::getSeries(plot_data, prefix + suffix[s++] );
            series.pushBack({timestamp, covariance[i*3 + j]});
        }
    }
}

inline void ParseCovariance(const std::string &prefix,
                            PlotDataMapRef &plot_data,
                            const std::array<double,36>& covariance,
                            double timestamp)
{
    static std::array<std::string,36> suffix = []()
    {
        std::array<std::string,36> out;
        int index = 0;
        for (int i = 0; i < 6; i++) {
            for (int j = i; j < 6; j++) {
                out[index++] = fmt::format( "[{},{}]", i, j );
            }
        }
        return out;
    }();

    size_t s = 0;
    for (int i = 0; i < 6; i++) {
        for (int j = i; j < 6; j++) {
            auto& series = MessageParserBase::getSeries(plot_data, prefix + suffix[s++] );
            series.pushBack({timestamp, covariance[i*6 + j]});
        }
    }
}
