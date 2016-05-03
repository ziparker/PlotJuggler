
include_directories( ./ ../  ../../common )

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_SHARED)


add_library(DataLoadCSV SHARED
    dataload_csv.cpp
    ../selectxaxisdialog.h
    ../selectxaxisdialog.ui
    )

target_link_libraries(DataLoadCSV common ${Qt5Widgets_LIBRARIES} )
