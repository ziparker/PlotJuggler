#*******************************************************************************
#  Copyright (c) 2015, 2020 logi.cals GmbH and others
#
#  All rights reserved. This program and the accompanying materials
#  are made available under the terms of the Eclipse Public License v2.0
#  and Eclipse Distribution License v1.0 which accompany this distribution.
#
#  The Eclipse Public License is available at
#     https://www.eclipse.org/legal/epl-2.0/
#  and the Eclipse Distribution License is available at
#    http://www.eclipse.org/org/documents/edl-v10.php.
#
#  Contributors:
#     Rainer Poisel - initial version
#     Genis Riera Perez - Add support for building debian package
#*******************************************************************************/

# Note: on OS X you should install XCode and the associated command-line tools

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12)
PROJECT("Eclipse Paho C" C)
MESSAGE(STATUS "CMake version: " ${CMAKE_VERSION})
MESSAGE(STATUS "CMake system name: " ${CMAKE_SYSTEM_NAME})

SET(CMAKE_SCRIPTS "${CMAKE_SOURCE_DIR}/cmake")
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules")

## build settings
file(READ version.major PAHO_VERSION_MAJOR)
file(READ version.minor PAHO_VERSION_MINOR)
file(READ version.patch PAHO_VERSION_PATCH)
SET(CLIENT_VERSION ${PAHO_VERSION_MAJOR}.${PAHO_VERSION_MINOR}.${PAHO_VERSION_PATCH})

INCLUDE(GNUInstallDirs)


IF (PAHO_HIGH_PERFORMANCE)
  ADD_DEFINITIONS(-DHIGH_PERFORMANCE=1)
ENDIF()

IF(WIN32)
  ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE -DWIN32_LEAN_AND_MEAN -MD)
ELSEIF(${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
  ADD_DEFINITIONS(-DOSX)
ENDIF()

## build options
SET(PAHO_WITH_SSL FALSE CACHE BOOL "Flag that defines whether to build ssl-enabled binaries too. ")
SET(PAHO_BUILD_STATIC TRUE)
SET(PAHO_BUILD_SHARED FALSE)

ADD_SUBDIRECTORY(src)


