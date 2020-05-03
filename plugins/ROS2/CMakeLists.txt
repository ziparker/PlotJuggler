
include_directories( ../ ros2_introspection/include)

# Qt related stuff
set(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

#############
add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_SHARED)
#############

SET( COMMON_SRC
   ros2_introspection/src/ros2_introspection.cpp
   ros2_introspection/src/stringtree.cpp

   dialog_select_ros_topics.h
   dialog_select_ros_topics.cpp
  )

QT5_WRAP_UI ( COMMON_UI_SRC
    dialog_select_ros_topics.ui
    dialog_with_itemlist.ui
    )

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(rcpputils REQUIRED)
find_package(fastcdr REQUIRED)
find_package(rosbag2_storage REQUIRED)

add_library( commonROS STATIC ${COMMON_SRC} ${COMMON_UI_SRC} )
target_link_libraries( commonROS
  ${Qt5Widgets_LIBRARIES}
  ${Qt5Xml_LIBRARIES}
  )
ament_target_dependencies( commonROS
    rclcpp
    rosbag2
    rcpputils
    fastcdr
    rosbag2_storage
)

#############
SET( DATALOAD_SRC
    DataLoadROS2/dataload_ros2.cpp
    ../../include/PlotJuggler/dataloader_base.h
    )

add_library( DataLoadROS2 SHARED ${DATALOAD_SRC}  )
target_link_libraries( DataLoadROS2  commonROS fastcdr)
ament_target_dependencies( DataLoadROS2
    rclcpp
    rosbag2
    rcpputils
    fastcdr
    rosbag2_storage
)

install(TARGETS DataLoadROS2
        DESTINATION bin )

