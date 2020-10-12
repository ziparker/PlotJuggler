
include_directories(
    ../../include
    paho.mqtt.c/src)

add_subdirectory(paho.mqtt.c)

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)

QT5_WRAP_UI ( UI_SRC  datastream_mqtt.ui  )

SET( SRC datastream_mqtt.cpp
    ../../include/PlotJuggler/datastreamer_base.h )

add_library(DataStreamMQTT SHARED ${SRC} ${UI_SRC}  )

target_link_libraries(DataStreamMQTT
    ${Qt5Widgets_LIBRARIES}
    paho-mqtt3c-static paho-mqtt3a-static
    fmt::fmt-header-only)





