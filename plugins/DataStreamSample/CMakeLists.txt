
include_directories( ./ ../  ../../common )

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_SHARED)

<<<<<<< HEAD
#QT5_WRAP_UI ( UI_SRC  ../common/selectlistdialog.ui  )
=======
#QT5_WRAP_UI ( UI_SRC  ../selectxaxisdialog.ui  )
>>>>>>> b821eb0b426188e0e1635851b6eb4b29d82b171e

SET( SRC
    datastream_sample.cpp
    ../datastreamer_base.h
    )

add_library(DataStreamSample SHARED ${SRC} ${UI_SRC}  )


target_link_libraries(DataStreamSample  ${Qt5Widgets_LIBRARIES} )

INSTALL(TARGETS DataStreamSample DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )
