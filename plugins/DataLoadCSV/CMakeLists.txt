
include_directories( ./ ../  ../../common )

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_SHARED)

QT5_WRAP_UI ( UI_SRC  ../../common/selectlistdialog.ui  )

SET( SRC
    dataload_csv.cpp
    ../../common/selectlistdialog.h
    )

add_library(DataLoadCSV SHARED ${SRC} ${UI_SRC}  )


target_link_libraries(DataLoadCSV  ${Qt5Widgets_LIBRARIES} )

INSTALL(TARGETS DataLoadCSV DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/PlotJuggler )
