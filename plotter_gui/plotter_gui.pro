TARGET = SuperPlotter
TEMPLATE = app

QT       += core gui xml
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

SOURCES = main.cpp\
        mainwindow.cpp \
    plotwidget.cpp \
    plotmatrix.cpp \
    removecurvedialog.cpp \
    plotmagnifier.cpp \
    busydialog.cpp \
    busytaskdialog.cpp \
    customtracker.cpp \
    curvecolorpick.cpp \
    plotdata_qwt.cpp

HEADERS  = mainwindow.h \
    plotwidget.h \
    plotmatrix.h \
    removecurvedialog.h \
    plotmagnifier.h \
    busydialog.h \
    busytaskdialog.h \
    customtracker.h \
    curvecolorpick.h \
    plotdata_qwt.cpp

FORMS  += mainwindow.ui \
    removecurvedialog.ui \
    busydialog.ui \
    curvecolorpick.ui

RESOURCES += \
    resource.qrc

DESTDIR = $$OUT_PWD/../bin
LIBDIR =  $$OUT_PWD/../lib

win32:CONFIG(release, debug|release):    LIBS += -L$$LIBDIR -lqwt  -lcommon  -lcolorwidgets
else:win32:CONFIG(debug, debug|release): LIBS += -L$$LIBDIR -lqwtd -lcommond -lcolorwidgetsd
else:unix:!macx:                         LIBS += -L$$LIBDIR -lqwt  -lcommon  -lcolorwidgets


INCLUDEPATH += $$PWD/../qwt/src
INCLUDEPATH += $$PWD/../common
INCLUDEPATH += $$PWD/../color_widgets/include

DEPENDPATH += $$PWD/../qwt/src
DEPENDPATH += $$PWD/../lib
DEPENDPATH += $$PWD/../common

win32-g++:CONFIG(release, debug|release): \
    PRE_TARGETDEPS += $$LIBDIR/libqwt.a \
                      $$LIBDIR/libcommon.a \
                      $$LIBDIR/libcolorwidgets.a

else:win32-g++:CONFIG(debug, debug|release): \
    PRE_TARGETDEPS += $$LIBDIR/libqwtd.a \
                      $$LIBDIR/libcommond.a \
                      $$LIBDIR/libcolorwidgetsd.a

else:win32:!win32-g++:CONFIG(release, debug|release):  \
    PRE_TARGETDEPS += $$LIBDIR/qwt.lib  \
                      $$LIBDIR/libcommon.lib \
                      $$LIBDIR/libcommond.lib

else:win32:!win32-g++:CONFIG(debug, debug|release):  \
    PRE_TARGETDEPS += $$LIBDIR/qwtd.lib \
                      $$LIBDIR/libcommond.lib \
                      $$LIBDIR/libcolorwidgetsd.lib

else:unix:!macx: PRE_TARGETDEPS += $$LIBDIR/libqwt.a \
                      $$LIBDIR/libcommon.a \
                      $$LIBDIR/libcolorwidgets.a



