# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST
#
# This file is part of sot-openhrp.
# sot-openhrp is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-openhrp is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-openhrp. If not, see <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/lapack.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/ros.cmake)

SET(PROJECT_NAME sot-openhrp)
SET(PROJECT_DESCRIPTION "OpenHRP plugin implementing the Stack of Tasks.")
SET(PROJECT_URL "https://github.com/jrl-umi3218/sot-openhrp")

SET(CUSTOM_HEADER_DIR "sot/openhrp")

# Define the location of the CMakeModules directory.
SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/CMakeModules)

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)

SETUP_PROJECT()

# Remove pedantic flag as Corba stubs won't compile with it.
STRING(REPLACE "-pedantic" "" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})

# Search for Boost.
SET(BOOST_COMPONENTS thread)
SEARCH_FOR_BOOST()

# Define additional options.
FIND_PATH(GRX_PREFIX
  # Make sure bin/DynamicsSimulator exists (to validate that it is
  # _really_ the prefix of an OpenHRP setup).
  bin/DynamicsSimulator
  HINTS /opt/grx3.0
  DOC "GRX software prefix (i.e. '/opt/grxX.Y')"
  NO_DEFAULT_PATH)

SET(ROBOT "" CACHE STRING "GRX robot name (HRP2JRL or HRP2LAAS)")
SET(ROBOT_PATH "${GRX_PREFIX}/${ROBOT}")

# Validate options.
IF(ROBOT STREQUAL "")
  MESSAGE(FATAL_ERROR
    "The ROBOT mandatory configuration variable is not set.")
ENDIF()
IF(NOT IS_DIRECTORY ${ROBOT_PATH})
  MESSAGE(FATAL_ERROR
    "The directory '${ROBOT_PATH}' does not exist."
    "Please check the ROBOT configuration variable.")
ENDIF()

# Log OpenHRP home and which robot will be used.
LIST(APPEND LOGGING_WATCHED_VARIABLES GRX_PREFIX ROBOT ROBOT_PATH)

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("omniORB4 >= 4.1.1")

ADD_REQUIRED_DEPENDENCY("dynamic-graph")
ADD_REQUIRED_DEPENDENCY("dynamic-graph-python")
ADD_REQUIRED_DEPENDENCY("sot-core")

# This is required by the Python prologue.
ADD_REQUIRED_DEPENDENCY("sot-dynamic")

ADD_ROSPACK_DEPENDENCY(dynamic_graph_bridge)

# Handle rpath.
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

ADD_SUBDIRECTORY(src)

ADD_SUBDIRECTORY(script)
ADD_SUBDIRECTORY(project)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
