TARGET = SuperPlotter
TEMPLATE = app

QT       += core gui xml
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

SOURCES = main.cpp\
        mainwindow.cpp \
    plotwidget.cpp \
    plotdata.cpp \
    plotmatrix.cpp \
    removecurvedialog.cpp \
    plotmagnifier.cpp \
    selectxaxisdialog.cpp \
    busydialog.cpp \
    busytaskdialog.cpp \
    customtracker.cpp

HEADERS  = mainwindow.h \
    plotwidget.h \
    plotdata.h \
    plotmatrix.h \
    removecurvedialog.h \
    plotmagnifier.h \
    selectxaxisdialog.h \
    busydialog.h \
    busytaskdialog.h \
    customtracker.h

FORMS  += mainwindow.ui \
    removecurvedialog.ui \
    selectxaxisdialog.ui \
    busydialog.ui

RESOURCES += \
    resource.qrc

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../qwt/lib/ -lqwt
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../qwt/lib/ -lqwtd
else:unix:!macx: LIBS += -L$$OUT_PWD/../qwt/lib/ -lqwt

INCLUDEPATH += $$PWD/../qwt/src
INCLUDEPATH += $$PWD/../qwt/lib

DEPENDPATH += $$PWD/../qwt/src
DEPENDPATH += $$PWD/../qwt/lib

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../qwt/lib/libqwt.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../qwt/lib/libqwtd.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../qwt/lib/qwt.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../qwt/lib/qwtd.lib
else:unix:!macx: PRE_TARGETDEPS += $$OUT_PWD/../qwt/lib/libqwt.a
