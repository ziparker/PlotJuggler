#-------------------------------------------------
#
# Project created by QtCreator 2016-02-26T23:42:47
#
#-------------------------------------------------

QT       += core gui xml

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += qwt

TARGET = Layout
TEMPLATE = app

INCLUDEPATH += C:\Qwt\include

SOURCES += main.cpp\
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

HEADERS  += mainwindow.h \
    plotwidget.h \
    plotdata.h \
    plotmatrix.h \
    removecurvedialog.h \
    plotmagnifier.h \
    selectxaxisdialog.h \
    busydialog.h \
    busytaskdialog.h \
    customtracker.h

FORMS    += mainwindow.ui \
    removecurvedialog.ui \
    selectxaxisdialog.ui \
    busydialog.ui

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../Qwt/lib/ -lqwt
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Qwt/lib/ -lqwtd

INCLUDEPATH += $$PWD/../../Qwt/include
DEPENDPATH += $$PWD/../../Qwt/include

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../Qwt/lib/libqwt.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../Qwt/lib/libqwtd.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../Qwt/lib/qwt.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../Qwt/lib/qwtd.lib
