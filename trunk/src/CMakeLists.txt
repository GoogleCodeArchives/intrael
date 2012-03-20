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


include_directories (${CMAKE_CURRENT_SOURCE_DIR} ${LIBFREENECT_INCLUDE_DIRS} ${LIBJPEG_INCLUDE_DIRS} )
LIST(APPEND SRC intrael.c md5.c)
IF(WIN32)
  set_source_files_properties(${SRC} PROPERTIES LANGUAGE CXX)
  set(THREADS_USE_PTHREADS_WIN32 true)
  find_package(Threads REQUIRED)
  include_directories(${THREADS_PTHREADS_INCLUDE_DIR})
ENDIF(WIN32)
find_package(Threads REQUIRED)
add_executable (intrael ${SRC})
if (WIN32)
  target_link_libraries(intrael ${LIBFREENECT_LIBRARIES} ${LIBJPEG_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} Ws2_32)
else(WIN32)
  IF(COMPILE_STATIC)
	target_link_libraries(intrael ${LIBFREENECT_STATIC} ${LIBJPEG_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${LIBUSB_1_LIBRARIES} m)
  ELSE(COMPILE_STATIC)
	target_link_libraries(intrael ${LIBFREENECT_LIBRARIES} ${LIBJPEG_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} m)
  endif(COMPILE_STATIC)
endif(WIN32)

install (TARGETS intrael DESTINATION bin)
