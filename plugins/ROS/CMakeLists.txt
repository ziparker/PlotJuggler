
include_directories( ../ ${catkin_INCLUDE_DIRS}  )

# Qt related stuff
set(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


#############
add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_SHARED)

SET( DATALOAD_SRC
	 DataLoadROS/dataload_ros.cpp
     DataLoadROS/dialog_select_ros_topics.h
     DataLoadROS/dialog_select_ros_topics.cpp
     ruleloaderwidget.h
     ruleloaderwidget.cpp
    )

QT5_WRAP_UI ( DATALOAD_UI_SRC
    DataLoadROS/dialog_select_ros_topics.ui
    ruleloaderwidget.ui
    )

SET( DATASTREAM_SRC
    DataStreamROS/datastream_ROS.cpp
    DataStreamROS/datastream_ROS.h
    DataStreamROS/rostopicselector.h
    DataStreamROS/rostopicselector.cpp
    ruleloaderwidget.h
    ruleloaderwidget.cpp
    )
    
QT5_WRAP_UI ( DATASTREAM_UI_SRC
    DataStreamROS/rostopicselector.ui
    ruleloaderwidget.ui )   
#############  

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


## Declare a C++ library
add_library( DataLoadROS SHARED ${DATALOAD_SRC} ${DATALOAD_UI_SRC} )
target_link_libraries( DataLoadROS  ${catkin_LIBRARIES} )

add_dependencies(DataLoadROS 
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
	)

add_library( DataStreamROS SHARED ${DATASTREAM_SRC} ${DATASTREAM_UI_SRC} )
target_link_libraries( DataStreamROS  ${catkin_LIBRARIES} )

add_dependencies(DataStreamROS 
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
	)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS DataLoadROS DataStreamROS 
        LIBRARY DESTINATION /usr/local/lib/PlotJuggler )


