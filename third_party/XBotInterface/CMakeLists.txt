#
#  Copyright (C) 2016 IIT-ADVR
#  Author: Arturo Laurenzi, Luca Muratore
#  email: arturo.laurenzi@iit.it, luca.muratore@iit.it
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>
#

cmake_minimum_required(VERSION 3.5)

project(XBotInterface VERSION 2.1.0)

# std c++11 flags
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
#set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# additional cmake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroOptionalBuild)
include(MacroInstallLib)

# find dependencies
find_package(kdl_parser REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdf REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

#include_directories(include
#                    ${EIGEN3_INCLUDE_DIRS}
#                    ${kdl_parser_INCLUDE_DIRS}
#                    ${urdfdom_INCLUDE_DIRS}
#                    ${urdf_INCLUDE_DIRS}
#                    ${Boost_INCLUDE_DIRS}
#                    ${orocos_kdl_INCLUDE_DIRS})

link_directories(/usr/lib/x86_64-linux-gnu/) # FIX FOR ROS HUMBLE

# define SOVERSION
set(${PROJECT_NAME}_SOVERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR})

# option to build tests
option(XBOTIFC_BUILD_TESTS "Compile tests" OFF)

# main xbotinterface library
add_library(XBotInterface SHARED
    src/ConfigOptions.cpp
    src/XBotInterface.cpp
    src/RobotInterface.cpp
    src/KinematicChain.cpp
    src/RobotChain.cpp
    src/ModelChain.cpp
    src/Joint.cpp
    src/Hand.cpp
    src/ModelInterface.cpp
    src/GenericSensor.cpp
    src/ForceTorqueSensor.cpp
    src/ImuSensor.cpp
    src/ControlMode.cpp
    src/Logger.cpp
    src/RtLog.cpp
    src/bprinter/table_printer.cpp
    )

target_include_directories(XBotInterface
    PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${kdl_parser_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )

target_link_libraries(XBotInterface
    PUBLIC
    yaml-cpp
    XBotCoreModel
    ${orocos_kdl_LIBRARIES}
    #PRIVATE
    #matio
    )

set_target_properties(XBotInterface PROPERTIES
    SOVERSION ${${PROJECT_NAME}_SOVERSION})

library_install(XBotInterface
    ${PROJECT_VERSION_MAJOR}
    ${PROJECT_VERSION_MINOR}
    ${PROJECT_VERSION_PATCH})

########################################################################
# srdfdom ADVR
########################################################################
add_subdirectory(srdfdom_advr)
########################################################################

########################################################################
# XBotCoreModel
########################################################################
add_library(XBotCoreModel SHARED src/XBotCoreModel.cpp)

target_link_libraries(XBotCoreModel
    PUBLIC
    yaml-cpp
    srdfdom_advr
    ${urdf_LIBRARIES}
    ${urdfdom_LIBRARIES}
    ${Boost_LIBRARIES}
    ${kdl_parser_LIBRARIES}
    kdl_parser # FIX FOR ROS HUMBLE
    )

target_include_directories(XBotCoreModel
    PUBLIC
    ${urdfdom_INCLUDE_DIRS}
    ${kdl_parser_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/srdfdom_advr/include>
    )
set_target_properties(XBotCoreModel PROPERTIES
    SOVERSION ${${PROJECT_NAME}_SOVERSION})

library_install(XBotCoreModel
    ${PROJECT_VERSION_MAJOR}
    ${PROJECT_VERSION_MINOR}
    ${PROJECT_VERSION_PATCH})
