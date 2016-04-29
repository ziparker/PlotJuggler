
include_directories( ../qwt/src ../common ../color_widgets/include)


SET( PLOTTER_SRC
    main.cpp
    mainwindow.cpp
    plotwidget.cpp
    plotmatrix.cpp
    removecurvedialog.cpp
    plotmagnifier.cpp
    busydialog.cpp
    busytaskdialog.cpp
    customtracker.cpp
    curvecolorpick.cpp
    plotdata_qwt.cpp

    mainwindow.ui
    removecurvedialog.ui
    busydialog.ui
    )

add_executable(SuperPlotter ${PLOTTER_SRC})

target_link_libraries(SuperPlotter
    ${Qt5Widgets_LIBRARIES}
    ${Qt5PrintSupport_LIBRARIES}
    ${Qt5Xml_LIBRARIES}
    ${Qt5Concurrent_LIBRARIES}
    ${Qt5Core_QTMAIN_LIBRARIES}
    common
    colorwidgets
    qwt_static
    )
