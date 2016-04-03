#-------------------------------------------------
#
# Project created by QtCreator 2016-04-02T17:09:56
#
#-------------------------------------------------



TARGET = common
TEMPLATE = lib
CONFIG += staticlib

SOURCES += plotdata.cpp
HEADERS += plotdata.h

INCLUDEPATH += $$PWD/../qwt/src

DESTDIR = $$OUT_PWD/../lib

CONFIG(debug, debug|release) {
     mac: TARGET = $$join(TARGET,,,_debug)
     win32: TARGET = $$join(TARGET,,,d)
 }

unix {
    target.path = /usr/lib
    INSTALLS += target
}
