
include_directories( ./ ../  ../../common )

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_SHARED)

QT5_WRAP_UI ( UI_SRC  ${CMAKE_SOURCE_DIR}/common/selectlistdialog.ui  )

SET( SRC
    dataload_csv.cpp
    ${CMAKE_SOURCE_DIR}/common/selectlistdialog.h
    )

add_library(DataLoadCSV SHARED ${SRC} ${UI_SRC}  )


target_link_libraries(DataLoadCSV  ${Qt5Widgets_LIBRARIES} )

INSTALL(TARGETS DataLoadCSV DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/PlotJuggler )
