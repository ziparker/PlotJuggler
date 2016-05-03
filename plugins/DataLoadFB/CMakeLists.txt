
include_directories( ./ ../  ../../common ./flatbuffers)

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_SHARED)


add_library(DataLoadFB SHARED
    dataload_fb.cpp
    ../selectxaxisdialog.h
    ../selectxaxisdialog.ui
    flatbuffers/flathash.cpp
    flatbuffers/util.cpp
    flatbuffers/reflection.cpp
    flatbuffers/idl_parser.cpp
    )

target_link_libraries(DataLoadFB common ${Qt5Widgets_LIBRARIES} )
