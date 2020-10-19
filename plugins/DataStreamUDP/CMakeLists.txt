
include_directories( ./ ../
    ../../include
    ../../common)

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)

QT5_WRAP_UI ( UI_SRC  udp_server.ui  )

SET( SRC udp_server.cpp
    ../../include/PlotJuggler/datastreamer_base.h )

add_library(DataStreamUDP SHARED ${SRC} ${UI_SRC}  )

target_link_libraries(DataStreamUDP
    ${Qt5Widgets_LIBRARIES})





