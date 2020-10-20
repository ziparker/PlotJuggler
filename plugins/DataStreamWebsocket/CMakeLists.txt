

find_package(Qt5 QUIET COMPONENTS WebSockets)

if( NOT Qt5WebSockets_FOUND)
    message(STATUS "Can't find Qt5WebSockets in your system. Have you tried [sudo apt-get install libqt5websockets5-dev] ?")
else()

    include_directories( ./ ../
        ../../include
        ../../common
        ${Qt5WebSockets_INCLUDE_DIRS})

    add_definitions(${QT_DEFINITIONS})
    add_definitions(-DQT_PLUGIN)

    QT5_WRAP_UI ( UI_SRC  websocket_server.ui  )

    SET( SRC websocket_server.cpp
        ../../include/PlotJuggler/datastreamer_base.h )

    add_library(DataStreamWebSocket SHARED ${SRC} ${UI_SRC}  )

    target_link_libraries(DataStreamWebSocket
        ${Qt5Widgets_LIBRARIES}
        Qt5::WebSockets)

    if(COMPILING_WITH_CATKIN)
        install(TARGETS DataStreamWebSocket
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
    elseif(COMPILING_WITH_AMENT)
        install(TARGETS DataStreamWebSocket
                DESTINATION  lib/${PROJECT_NAME} )
    else()
        install(TARGETS DataLoadCSV DESTINATION bin  )
    endif()

endif()



