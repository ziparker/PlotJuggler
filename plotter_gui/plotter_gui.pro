TARGET = SuperPlotter
TEMPLATE = app

QT       += core gui xml
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

SOURCES = main.cpp\
        mainwindow.cpp \
    plotwidget.cpp \
    plotmatrix.cpp \
    removecurvedialog.cpp \
    plotmagnifier.cpp \
    busydialog.cpp \
    busytaskdialog.cpp \
    customtracker.cpp

HEADERS  = mainwindow.h \
    plotwidget.h \
    plotmatrix.h \
    removecurvedialog.h \
    plotmagnifier.h \
    busydialog.h \
    busytaskdialog.h \
    customtracker.h

FORMS  += mainwindow.ui \
    removecurvedialog.ui \
    busydialog.ui

RESOURCES += \
    resource.qrc

DESTDIR = $$OUT_PWD/../bin

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../lib/ -lqwt -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../lib/ -lqwtd -lcommond
else:unix:!macx: LIBS += -L$$OUT_PWD/../lib/ -lqwt -lcommon


INCLUDEPATH += $$PWD/../qwt/src

DEPENDPATH += $$PWD/../qwt/src
DEPENDPATH += $$PWD/../lib

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../lib/libqwt.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../lib/libqwtd.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../lib/qwt.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../lib/qwtd.lib
else:unix:!macx: PRE_TARGETDEPS += $$OUT_PWD/../lib/libqwt.a


INCLUDEPATH += $$PWD/../common
DEPENDPATH += $$PWD/../common

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../common/lib/libcommon.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../common/lib/libcommon.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../common/lib/common.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../common/lib/common.lib
else:unix:!macx: PRE_TARGETDEPS += $$OUT_PWD/../common/libcommon.a
