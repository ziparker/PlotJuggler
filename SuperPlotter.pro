#-------------------------------------------------
#
# Project created by QtCreator 2016-02-26T23:42:47
#
#-------------------------------------------------

QT       += core gui xml

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += qwt

TARGET = SuperPlotter
TEMPLATE = app

QWT_INSTALL_DIR = /usr/local/qwt-sys

INCLUDEPATH += $${QWT_INSTALL_DIR}/include

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

LIBS += -L$${QWT_INSTALL_DIR}/lib/ -lqwt



INCLUDEPATH += $${QWT_INSTALL_DIR}/include
DEPENDPATH += $${QWT_INSTALL_DIR}/include

PRE_TARGETDEPS += $${QWT_INSTALL_DIR}/lib/libqwt.so


RESOURCES += \
    resource.qrc
