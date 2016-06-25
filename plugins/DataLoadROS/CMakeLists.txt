
include_directories( ./ ../  ../../common )

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_SHARED)

QT5_WRAP_UI ( UI_SRC  dialog_select_ros_topics.ui  )

SET( SRC
    dataload_ros.cpp
    ros-type-parser.h
    ros-type-parser.cpp
    dialog_select_ros_topics.h
    dialog_select_ros_topics.cpp
    )

SET ( ROS_DIRECTORY /opt/ros/kinetic/)

include_directories( ${ROS_DIRECTORY}/include)

add_library(DataLoadROS SHARED ${SRC} ${UI_SRC}  )


target_link_libraries(DataLoadROS  ${Qt5Widgets_LIBRARIES}
    ${ROS_DIRECTORY}/lib/libroscpp.so
    ${ROS_DIRECTORY}/lib/librosbag.so
    ${ROS_DIRECTORY}/lib/librosbag_storage.so
    ${ROS_DIRECTORY}/lib/librostime.so
    ${ROS_DIRECTORY}/lib/libroscpp_serialization.so
    )

INSTALL(TARGETS DataLoadROS DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )
