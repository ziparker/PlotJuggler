# Copyright (C) 2013-2015 Mattia Basaglia
#
#
# This software is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with Color Widgets.  If not, see <http://www.gnu.org/licenses/>.

CONFIG += c++11

INCLUDEPATH += $$PWD/src $$PWD/include

SOURCES += \
    $$PWD/src/color_preview.cpp \
    $$PWD/src/color_wheel.cpp \
    $$PWD/src/color_dialog.cpp \
    $$PWD/src/color_selector.cpp \
    $$PWD/src/color_palette.cpp \
    $$PWD/src/swatch.cpp \
    $$PWD/src/color_utils.cpp \
   $$PWD/src/color_names.cpp

HEADERS += \
    $$PWD/include/color_wheel.hpp \
    $$PWD/include/color_preview.hpp \
    $$PWD/include/color_dialog.hpp \
    $$PWD/include/color_selector.hpp \
    $$PWD/include/colorwidgets_global.hpp \
    $$PWD/include/color_palette.hpp \
    $$PWD/include/swatch.hpp \
    $$PWD/src/color_utils.hpp \
    $$PWD/include/color_names.hpp

FORMS += \
    $$PWD/src/color_dialog.ui

RESOURCES += \
    $$PWD/src/color_widgets.qrc

