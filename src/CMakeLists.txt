# Copyright 2010, 2011, François Bleibel, Thomas Moulard Olivier
# Stasse, JRL, CNRS/AIST
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

include(../cmake/python.cmake)
include(../cmake/idl.cmake)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

# Add OpenHRP and robot headers to include directory.
INCLUDE_DIRECTORIES(
  ${GRX_PREFIX}/OpenHRP/Controller/corba
  ${GRX_PREFIX}/include
  ${ROBOT_PATH}
  )

# Create and install python module
GENERATE_IDL_FILE(OpenHRPCommon ${GRX_PREFIX}/OpenHRP/Common/corba)
GENERATE_IDL_FILE(HRPcontroller ${GRX_PREFIX}/include/idl)
GENERATE_IDL_FILE(ViewSimulator ${GRX_PREFIX}/include/idl)

# OpenHRP generates stubs headers with '.h' suffix but they are
# generated with '.hh' suffix. Provide an alternative version to
# please OpenHRP headers.
ADD_CUSTOM_COMMAND(
  OUTPUT OpenHRPCommon.h
  COMMAND cp
  ARGS OpenHRPCommon.hh OpenHRPCommon.h
  MAIN_DEPENDENCY OpenHRPCommon.hh
  )
ADD_CUSTOM_COMMAND(
  OUTPUT HRPcontroller.h
  COMMAND cp
  ARGS HRPcontroller.hh HRPcontroller.h
  MAIN_DEPENDENCY HRPcontroller.hh
  )

# Declare the shared library containing the OpenHRP specific entity.
ADD_LIBRARY(openhrp
  SHARED
  stack-of-tasks.cc
  stack-of-tasks.hh
  OpenHRPCommon.hh
  OpenHRPCommonSK.cc
  HRPcontroller.hh
  HRPcontrollerSK.cc
  ViewSimulator.hh
  ViewSimulatorSK.cc
  )

SET_TARGET_PROPERTIES(
  openhrp PROPERTIES SOVERSION ${PROJECT_VERSION})

INSTALL(TARGETS openhrp DESTINATION lib)

# Declare library dependencies.
PKG_CONFIG_USE_DEPENDENCY(openhrp "omniORB4")
PKG_CONFIG_USE_DEPENDENCY(openhrp "dynamic-graph")
PKG_CONFIG_USE_DEPENDENCY(openhrp "sot-core")

# Generate Python module.
DYNAMIC_GRAPH_PYTHON_MODULE("sot/openhrp" openhrp wrap)

# Configure prologue to embed robot model directory.
CONFIG_FILES(dynamic_graph/sot/openhrp/prologue.py)

# Install Python files.
INSTALL(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/dynamic_graph/sot/openhrp/prologue.py
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/openhrp/test_reach.py
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/openhrp/log.py
  DESTINATION
  ${PYTHON_SITELIB}/dynamic_graph/sot/openhrp
  )

# Install the sample openhrp shell
INSTALL(FILES
  openhrp-shell
  DESTINATION bin
  PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE
  GROUP_READ GROUP_EXECUTE
  WORLD_READ WORLD_EXECUTE
  )


# Compile the OpenHRP plug-in.
ADD_LIBRARY(StackOfTasks SHARED plugin.cc)

# Remove 'lib' prefix in library filename.
SET_TARGET_PROPERTIES(StackOfTasks PROPERTIES PREFIX ""
  INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# Declare plug-in dependencies.
TARGET_LINK_LIBRARIES(StackOfTasks openhrp)

ROSPACK_USE_DEPENDENCY(StackOfTasks dynamic_graph_bridge)
ROSPACK_USE_DEPENDENCY(openhrp dynamic_graph_bridge)

INSTALL(TARGETS StackOfTasks DESTINATION "${ROBOT_PATH}/bin")


LIST(APPEND LOGGING_WATCHED_TARGETS openhrp StackOfTasks)
