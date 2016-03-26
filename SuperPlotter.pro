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

INCLUDEPATH += C:\Qwt-6.1.3-svn\include

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

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../Qwt-6.1.3-svn/lib/ -lqwt
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Qwt-6.1.3-svn/lib/ -lqwtd

INCLUDEPATH += $$PWD/../../Qwt-6.1.3-svn/include
DEPENDPATH += $$PWD/../../Qwt-6.1.3-svn/include

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../Qwt-6.1.3-svn/lib/libqwt.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../Qwt-6.1.3-svn/lib/libqwtd.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../Qwt-6.1.3-svn/lib/qwt.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../Qwt-6.1.3-svn/lib/qwtd.lib
