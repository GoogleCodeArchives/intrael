######################################################################################
# License
######################################################################################

# This file is part of the Intrael Project which provides computer vision for the web.
#Copyright (C) 2011 Yannis Gravezas. You can contact the author at wizgrav@gmail.com.

#This program is free software: you can redistribute it and/or modify
#it under the terms of the Affero GNU General Public License as published
#by the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.

#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#Affero GNU General Public License v3 for more details.

#You should have received a copy of the Affero GNU General Public License v3
#along with this program.  If not, see <http://www.gnu.org/licenses/gpl-3.0.txt>.


######################################################################################
# CMake directives
######################################################################################

#Require 2.6 or higher. 

cmake_minimum_required(VERSION 2.6)

######################################################################################
# Project declaration and options
######################################################################################

PROJECT(Intrael)

set (PROJECT_VER_MAJOR 2)
set (PROJECT_VER_MINOR 0)
set (PROJECT_VER_PATCH 0)
set (PROJECT_VER 
  "${PROJECT_VER_MAJOR}.${PROJECT_VER_MINOR}.${PROJECT_VER_PATCH}")

OPTION(COMPILE_SSE "Use x86 SSE instruction set" OFF)
OPTION(COMPILE_SSE2 "Use x86 SSE2 instruction set" ON)
OPTION(COMPILE_NEON "Use Arm NEON instruction set" OFF)


######################################################################################
# CMake Modules
######################################################################################

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

# Find the host operating system and architecture
include (FindOS)
# Set up installation directories
include (SetupDirectories)

# Find packages needed to build library
find_package(libfreenect REQUIRED)
find_package(libjpeg REQUIRED)
if (WIN32)
  set(THREADS_USE_PTHREADS_WIN32 true)
endif()
find_package(Threads REQUIRED)

######################################################################################
# CMake 
######################################################################################

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
SET(DOC_OUTPUT_PATH ${CMAKE_BINARY_DIR}/doc)

if(CMAKE_C_FLAGS STREQUAL "")
  set(CMAKE_C_FLAGS "-O3 -g")
endif()
add_definitions(-Wall)

IF ( COMPILE_SSE )
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -msse")
        add_definitions(-DUSE_SSE)
ENDIF ( COMPILE_SSE )

IF ( COMPILE_SSE2 )
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -msse2")
	add_definitions(-DUSE_SSE2)
ENDIF ( COMPILE_SSE2 )

IF ( COMPILE_NEON )
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -mfpu=neon")
	add_definitions(-DUSE_NEON)
ENDIF ( COMPILE_NEON )



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
