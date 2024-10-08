## Copyright (C) 2020  I. Bogoslavskyi, C. Stachniss
##
## Permission is hereby granted, free of charge, to any person obtaining a
## copy of this software and associated documentation files (the "Software"),
## to deal in the Software without restriction, including without limitation
## the rights to use, copy, modify, merge, publish, distribute, sublicense,
## and/or sell copies of the Software, and to permit persons to whom the
## Software is furnished to do so, subject to the following conditions:
##
## The above copyright notice and this permission notice shall be included in
## all copies or substantial portions of the Software.
##
## THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
## IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
## FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
## AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
## LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
## FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
## DEALINGS IN THE SOFTWARE.

cmake_minimum_required(VERSION 3.5)
project(depth_clustering_for_rail)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(DistVersion.cmake)
system_info(DISTRO)
message(STATUS "DISTRO: ${DISTRO}")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# cmake_policy(SET CMP0045 OLD)

SET(CMAKE_MODULE_PATH
      ${CMAKE_MODULE_PATH}
      "${PROJECT_SOURCE_DIR}/cmake_modules"
      "/usr/share/cmake-2.8/Modules"  ## Hack for travis
    )

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY install/depth_clustering_for_rail/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY install/depth_clustering_for_rail/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY install/depth_clustering_for_rail/bin)

include(CTest)
IF(IS_DIRECTORY "/usr/src/gtest/")
  MESSAGE(STATUS "Found google test sources in /usr/src/gtest/")

  ADD_SUBDIRECTORY(/usr/src/gtest/ gtest)

  # mimick the behaviour of find_package(GTest)
  SET(GTEST_FOUND TRUE)
  SET(GTEST_BOTH_LIBRARIES gtest gtest_main)
  SET(GTEST_LIBRARIES gtest)
  SET(GTEST_MAIN_LIBRARIES gtest_main)
ELSE()
  find_package(GTest)
ENDIF()

set(Boost_USE_STATIC_LIBS OFF)

option(BUILD_WITH_COVERAGE "Enable compilation with coverage information." OFF)
if(BUILD_WITH_COVERAGE)
  message(STATUS "Enabled generation of code coverage information with gcc.")
  set(CMAKE_CXX_FLAGS "-fprofile-arcs -ftest-coverage ${CMAKE_CXX_FLAGS}")
endif()

# find correct qt version
if(${DISTRO} MATCHES "14.04")
  find_package(Qt4 REQUIRED COMPONENTS QtCore QtXml QtOpenGL QtGui QtMultimedia)
  include(${QT_USE_FILE})
  include_directories(${QT_INCLUDES})
  set(MY_QT_LIBRARIES
      Qt4::QtCore
      Qt4::QtXml
      Qt4::QtOpenGL
      Qt4::QtGui)
else()
  find_package(Qt5 REQUIRED COMPONENTS Core Xml OpenGL Gui Widgets)
  include_directories(${Qt5Core_INCLUDE_DIRS}
                      ${Qt5Xml_INCLUDE_DIRS}
                      ${Qt5Gui_INCLUDE_DIRS}
                      ${Qt5Widgets_INCLUDE_DIRS}
                      ${Qt5OpenGL_INCLUDE_DIRS})
  set(MY_QT_LIBRARIES
      ${Qt5Widgets_LIBRARIES}
      ${Qt5Core_LIBRARIES}
      ${Qt5Gui_LIBRARIES}
      ${Qt5Xml_LIBRARIES}
      ${Qt5OpenGL_LIBRARIES})
endif()

# some status messages
message(STATUS "Linking against Qt libs: ${MY_QT_LIBRARIES}")
message(STATUS "Linking against QGlViewer lib: ${QGLVIEWER_LIBRARY}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(depth_clustering_for_rail_interfaces REQUIRED)
find_package(foxglove_msgs REQUIRED)

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem regex program_options)
find_package(PCL QUIET COMPONENTS common io)
find_package(QGLViewer REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# add some useful constants to configuration
configure_file(config/cmake_config.h.in
               ${PROJECT_SOURCE_DIR}/config/cmake_config.h)
include_directories(config)

if (${PCL_FOUND})
  message(STATUS "PCL is found. PCL-related code will be built.")
  if (${CMAKE_BUILD_TYPE} MATCHES "Debug" AND ${DISTRO} MATCHES "14.04")
    message(FATAL_ERROR
            "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
            "You are building PCL in Debug mode with C++11 enabled.\n"
            "This causes segmentation fault due to a bug in PCL.\n"
            "Build in Release mode to avoid this bug.\n"
            "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
  endif()
  add_definitions(-DPCL_FOUND)
else()
  message(WARNING "PCL NOT FOUND! PCL-related code WILL NOT be built")
endif()

if(${rclcpp_FOUND})
  message(STATUS "ROS found, building ROS related parts")
  set(ROS_FOUND YES)
else()
  message(WARNING "ROS NOT FOUND. NOT building ROS related parts!")
  set(ROS_FOUND NO)
endif()

include_directories(
  include
  src
  # for QT moc files
  ${PROJECT_BINARY_DIR}/src
  # dependencies we have no control over
  SYSTEM ${EIGEN3_INCLUDE_DIR}
  SYSTEM ${Boost_INCLUDE_DIRS}
  SYSTEM ${PCL_INCLUDE_DIRS}
  SYSTEM ${OpenCV_INCLUDE_DIRS}
  SYSTEM ${OpenGL_INCLUDE_DIRS}
  SYSTEM ${QGLVIEWER_INCLUDE_DIR}
)

add_subdirectory(src)
add_subdirectory(examples)

message(STATUS "BUILD_TESTING: ${BUILD_TESTING}")
if (${BUILD_TESTING})
  add_subdirectory(test)
endif()

ament_package()