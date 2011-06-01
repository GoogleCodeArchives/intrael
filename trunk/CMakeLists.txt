######################################################################################
# License
######################################################################################

# This file is part of the Intrael Project. http://www.infrael.com
#
# Copyright (c) 2011 Yannis Gravezas.
#
# This code is licensed to you under the terms of the GNU General Public License,
# version 3.0. See the GPL3 files for the text of the license,
# or visit http://www.gnu.org/licenses/gpl-3.0.txt
#
# If you redistribute this file in source form, modified or unmodified, you
# must keep the copyright notice intact and include
#
# Binary distributions must follow the binary distribution requirements of
# the GPL3 License.

######################################################################################
# CMake directives
######################################################################################

#Require 2.6 or higher. 

cmake_minimum_required(VERSION 2.6)

######################################################################################
# Project declaration and options
######################################################################################

PROJECT(Infrael)

set (PROJECT_VER_MAJOR 0)
set (PROJECT_VER_MINOR 0)
set (PROJECT_VER_PATCH 1)
set (PROJECT_VER 
  "${PROJECT_VER_MAJOR}.${PROJECT_VER_MINOR}.${PROJECT_VER_PATCH}")

IF(PROJECT_OS_LINUX)
	OPTION(BUILD_CPACK "Build an RPM or DEB using CPack" OFF)
ENDIF(PROJECT_OS_LINUX)

######################################################################################
# CMake Modules
######################################################################################

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

# Find the host operating system and architecture
include (FindOS)
# Set up installation directories
include (SetupDirectories)

# Find packages needed to build library
IF(WIN32)
  set(THREADS_USE_PTHREADS_WIN32 true)
ENDIF(WIN32)
find_package(Threads REQUIRED)
IF(WIN32)
	include_directories(${THREADS_PTHREADS_INCLUDE_DIR})
ENDIF(WIN32)
find_package(libfreenect REQUIRED)
include_directories(${LIBFREENECT_INCLUDE_DIRS})


######################################################################################
# CMake 
######################################################################################

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
SET(DOC_OUTPUT_PATH ${CMAKE_BINARY_DIR}/doc)

if(CMAKE_C_FLAGS STREQUAL "")
  set(CMAKE_C_FLAGS "-O3 -ffast-math")
endif()
add_definitions(-Wall)


add_subdirectory (src)

######################################################################################
# Extras
######################################################################################

# Create an uninstall target
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/UninstallTarget.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/UninstallTarget.cmake"
  IMMEDIATE @ONLY)

add_custom_target(uninstall
  COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/UninstallTarget.cmake)

# Create Debian/RPM Packages
# after make, use "fakeroot cpack" in the build Dir to complete

IF ( BUILD_CPACK )
  set(CPACK_PACKAGE_DESCRIPTION "Intrael")
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Computer vision for the web")
  set(CPACK_PACKAGE_NAME "Intrael")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libfreenect")

  set(CPACK_PACKAGE_CONTACT "Yannis Gravezas <wizgrav@intrael.com>")
  #set(CPACK_PACKAGE_VENDOR "")
  set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VER_MAJOR})
  set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VER_MINOR})
  set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VER_PATCH})
  set(VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")

  set(CPACK_GENERATOR "DEB;RPM;")
  set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}-${CMAKE_SYSTEM_PROCESSOR}")

  include(CPack)

  INSTALL(FILES "${CMAKE_BINARY_DIR}/bin/intrael" DESTINATION ${PROJECT_BINARY_INSTALL_DIR})
  INSTALL(FILES "GPL3.txt" DESTINATION "share/doc/${CPACK_PACKAGE_NAME}")
  INSTALL(FILES "README.asciidoc" DESTINATION "share/doc/${CPACK_PACKAGE_NAME}")
  INSTALL(FILES "MANUAL.asciidoc" DESTINATION "share/doc/${CPACK_PACKAGE_NAME}")

ENDIF ( BUILD_CPACK )

