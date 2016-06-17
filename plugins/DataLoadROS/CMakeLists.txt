
include_directories( ./ ../  ../../common )

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_SHARED)

QT5_WRAP_UI ( UI_SRC  ../selectxaxisdialog.ui  )

SET( SRC
    dataload_ros.cpp
    ros-type-parser.h
    ros-type-parser.cpp
    ../selectxaxisdialog.h
    )

SET ( ROS_LIBRARIES /opt/ros/indigo/lib/)


add_library(DataLoadROS SHARED ${SRC} ${UI_SRC}  )


target_link_libraries(DataLoadROS  ${Qt5Widgets_LIBRARIES}
    ${ROS_LIBRARIES}/libroscpp.so
    ${ROS_LIBRARIES}/librosbag.so
    ${ROS_LIBRARIES}/librosbag_storage.so
    ${ROS_LIBRARIES}/librostime.so
    ${ROS_LIBRARIES}/libroscpp_serialization.so
    )

INSTALL(TARGETS DataLoadROS DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )
