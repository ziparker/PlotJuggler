TEMPLATE=lib
CONFIG += dll
QT += core gui
DEFINES += QTCOLORWIDGETS_LIBRARY

TARGET=ColorWidgets

greaterThan(QT_MAJOR_VERSION, 4) {
	QT += widgets
}


include(color_widgets.pri)

CONFIG(debug, debug|release) {
     mac: TARGET = $$join(TARGET,,,_debug)
     win32: TARGET = $$join(TARGET,,,d)
 }

DESTDIR = $$OUT_PWD/../lib

unix {
    LIB_TARGET = lib$${TARGET}.so
}
win32 {
    LIB_TARGET = $${TARGET}.dll
}

isEmpty(PREFIX) {
    PREFIX = /usr/local
}



