project(SuperPlotterCmake)
cmake_minimum_required(VERSION 2.8)

CMAKE_POLICY(SET CMP0020 NEW)

find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5PrintSupport)
find_package(Qt5Concurrent)
find_package(Qt5Xml)

set(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")

include_directories(
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Concurrent_INCLUDE_DIRS}
    ${Qt5PrintSupport_INCLUDE_DIRS}
    ${Qt5Xml_INCLUDE_DIRS}
    )


add_subdirectory(common)
add_subdirectory(color_widgets)
add_subdirectory(qwt/src)
add_subdirectory( plotter_gui )



