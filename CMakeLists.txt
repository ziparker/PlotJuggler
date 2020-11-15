cmake_minimum_required(VERSION 3.5)

cmake_policy(SET CMP0048 OLD)
PROJECT(plotjuggler)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")

add_definitions(
    -DPJ_MAJOR_VERSION="3"
    -DPJ_MINOR_VERSION="0"
    -DPJ_PATCH_VERSION="0" )

# http://answers.ros.org/question/230877/optionally-build-a-package-with-catkin/
if( CATKIN_DEVEL_PREFIX OR catkin_FOUND OR CATKIN_BUILD_BINARY_PACKAGE)
    set(COMPILING_WITH_CATKIN 1)

    message(STATUS "---------------------------------------------------------------------")
    message(STATUS "PlotJuggler is being built using CATKIN. ROS plugins will be compiled")
    message(STATUS "---------------------------------------------------------------------")

    find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES} )

    catkin_package(
        INCLUDE_DIRS  include
        LIBRARIES  plotjuggler_plugin_base
    )

elseif( DEFINED ENV{AMENT_PREFIX_PATH})
    set(COMPILING_WITH_AMENT 1)
    
    message(STATUS "---------------------------------------------------------------------")
    message(STATUS "PlotJuggler is being built using AMENT. ROS2 plugins will be compiled")
    message(STATUS "---------------------------------------------------------------------")
else()
    message(STATUS "---------------------------------------------------------------------")
    message(STATUS "PlotJuggler is being WITHOUT any ROS support")
    message(STATUS "---------------------------------------------------------------------")
endif()

cmake_policy (SET CMP0020 NEW)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)

if(NOT CMAKE_VERSION VERSION_LESS 3.1)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
endif()

if (NOT WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fno-omit-frame-pointer")
endif()

if(APPLE AND EXISTS /usr/local/opt/qt5)
    # Homebrew installs Qt5 (up to at least 5.9.1) in
    # /usr/local/qt5, ensure it can be found by CMake since
    # it is not in the default /usr/local prefix.
    # source: https://github.com/Homebrew/homebrew-core/issues/8392#issuecomment-325226494
    list(APPEND CMAKE_PREFIX_PATH "/usr/local/opt/qt5")
    set(CMAKE_MACOSX_RPATH 1)
endif()


find_package(Qt5 REQUIRED COMPONENTS
    Core
    Widgets
    PrintSupport
    Concurrent
    Xml
    Svg
    OpenGL
    WebSockets)

set( QT_LINK_LIBRARIES
    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport
    Qt5::Xml
    Qt5::Concurrent
    Qt5::Svg
    Qt5::OpenGL
    Qt5::WebSockets
    )

if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to RelWithDebInfo")
    set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

IF (NOT WIN32)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -fPIC")
ENDIF()

find_package(ZMQ)

include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Concurrent_INCLUDE_DIRS}
    ${Qt5PrintSupport_INCLUDE_DIRS}
    ${Qt5Xml_INCLUDE_DIRS}
    ${Qt5WebSockets_INCLUDE_DIRS}
    ${Qt5Svg_INCLUDE_DIRS}
    ${Qt5OpenGL_INCLUDE_DIRS}
    ./include
    ./3rdparty/
    ./3rdparty/color_widgets/include
    ./3rdparty/Qt-Advanced-Docking/include
    ./3rdparty/paho.mqtt.c/src
    ./3rdparty/qwt/src
    ./3rdparty/sol
    ./3rdparty/lua-5.3.4/src
)

#########################  INSTALL ####################################

if(COMPILING_WITH_CATKIN)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})

    install(DIRECTORY include/
          DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
          PATTERN ".svn" EXCLUDE
        )
	 add_definitions(-DCOMPILED_FOR_ROS)

elseif(COMPILING_WITH_AMENT)
    find_package(ament_cmake REQUIRED)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
	 add_definitions(-DCOMPILED_FOR_ROS)
else()
    set(CMAKE_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()

#############################################################

if (NOT WIN32)
  add_subdirectory( 3rdparty/backward-cpp )
endif()

add_subdirectory( 3rdparty/color_widgets )
add_subdirectory( 3rdparty/Qt-Advanced-Docking )
add_subdirectory( 3rdparty/qwt/src )

qt5_wrap_cpp(PLOTJUGGLER_BASE_MOCS
  include/PlotJuggler/dataloader_base.h
  include/PlotJuggler/datastreamer_base.h
  include/PlotJuggler/statepublisher_base.h
  include/PlotJuggler/transform_function.h
)

add_library( plotjuggler_plugin_base STATIC ${PLOTJUGGLER_BASE_MOCS})
target_link_libraries( plotjuggler_plugin_base PRIVATE
    ${Qt5Widgets_LIBRARIES}
    ${catkin_LIBRARIES} )

add_subdirectory( 3rdparty/lua-5.3.4 )

SET(PAHO_ENABLE_TESTING FALSE CACHE BOOL "Build tests and run" FORCE)
SET(PAHO_ENABLE_CPACK FALSE CACHE BOOL "Enable CPack" FORCE)
SET(PAHO_BUILD_STATIC TRUE CACHE BOOL "Build static library" FORCE)
SET(PAHO_BUILD_SHARED FALSE CACHE BOOL "Build shared library" FORCE)
add_subdirectory( 3rdparty/paho.mqtt.c )

add_subdirectory( src )

add_subdirectory( plugins/DataLoadCSV )
add_subdirectory( plugins/DataLoadULog )
add_subdirectory( plugins/DataStreamSample )
add_subdirectory( plugins/DataStreamWebsocket )
add_subdirectory( plugins/DataStreamMQTT )
add_subdirectory( plugins/DataStreamUDP )

if( ZMQ_FOUND )
  add_subdirectory( plugins/DataStreamZMQ )
endif()


if(COMPILING_WITH_AMENT)
    ament_package()
endif()

#############################################################

set(DESKTOP_ICON_DIR "${CMAKE_INSTALL_PREFIX}/share/icons")

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/PlotJuggler.desktop
        DESTINATION  "${CMAKE_INSTALL_PREFIX}/share/applications"  )

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/plotjuggler.svg
        DESTINATION ${DESKTOP_ICON_DIR}  )

