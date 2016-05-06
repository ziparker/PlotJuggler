
include_directories( ./ ../  ../../common ./flatbuffers)

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_SHARED)

QT5_WRAP_UI ( UI_SRC  ../selectxaxisdialog.ui  )


SET( SRC
    dataload_fb.cpp
    ../selectxaxisdialog.h
    flatbuffers/flathash.cpp
    flatbuffers/util.cpp
    flatbuffers/reflection.cpp
    flatbuffers/idl_parser.cpp
    )

add_library(DataLoadFB SHARED  ${SRC} ${UI_SRC}  )

target_link_libraries(DataLoadFB common ${Qt5Widgets_LIBRARIES} )
