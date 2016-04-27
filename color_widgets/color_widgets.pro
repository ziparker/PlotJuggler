

TARGET = colorwidgets

TEMPLATE=lib
CONFIG += staticlib c++11
QT += core gui

#DEFINES += QTCOLORWIDGETS_LIBRARY


greaterThan(QT_MAJOR_VERSION, 4) {
	QT += widgets
}

INCLUDEPATH += $$PWD/src $$PWD/include

SOURCES += \
    $$PWD/src/color_preview.cpp \
    $$PWD/src/color_wheel.cpp \
    $$PWD/src/color_palette.cpp \
    $$PWD/src/swatch.cpp \
    $$PWD/src/color_utils.cpp \
   $$PWD/src/color_names.cpp

HEADERS += \
    $$PWD/include/color_wheel.hpp \
    $$PWD/include/color_preview.hpp \
    $$PWD/include/colorwidgets_global.hpp \
    $$PWD/include/color_palette.hpp \
    $$PWD/include/swatch.hpp \
    $$PWD/include/color_utils.hpp \
    $$PWD/include/color_names.hpp

FORMS += \
  #   $$PWD/src/color_dialog.ui

RESOURCES += \
    $$PWD/src/color_widgets.qrc

CONFIG(debug, debug|release) {
     mac: TARGET = $$join(TARGET,,,_debug)
     win32: TARGET = $$join(TARGET,,,d)
 }

DESTDIR = $$OUT_PWD/../lib


isEmpty(PREFIX) {
    PREFIX = /usr/local
}



