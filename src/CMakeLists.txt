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

# Add jrl-mal compilation flags and link to library
# libMatrixAbstractLayer.so
include(../cmake/python.cmake)
include(../cmake/idl.cmake)

ADD_DEFINITIONS(${JRL_MAL_CFLAGS})

# -------------------------------------------------------------------
# --- UTILS ---------------------------------------------------------
# -------------------------------------------------------------------
MACRO(LIST2STRING _VAR_ARG _LIST_ARG)
  SET(${_VAR_ARG})
  SET(_FIRST_STEP 1)
  FOREACH(_ITEM ${ARGV})
    IF(_FIRST_STEP)
      SET(_FIRST_STEP 0)
    ELSE(_FIRST_STEP)
      SET (${_VAR_ARG} "${${_VAR_ARG}} ${_ITEM}")
    ENDIF(_FIRST_STEP)
  ENDFOREACH(_ITEM)
ENDMACRO(LIST2STRING)


# --- OpenHRP 3 -----------------------------------------------------------
IF(OPENHRP_VERSION_3)

  SET(HRP_ROBOT_SPEC HRP2JRL CACHE STRING
    "Robot name, to resolve specific path in OpenHRP.")

  INCLUDE_DIRECTORIES(
    BEFORE ${CMAKE_CURRENT_BINARY_DIR}
    )
  # Create and install python module
  GENERATE_IDL_FILE(OpenHRPCommon ${OPENHRP_HOME}/OpenHRP/Common/corba)
  GENERATE_IDL_FILE(HRPcontroller ${OPENHRP_HOME}/include/idl)
  GENERATE_IDL_FILE(ViewSimulator ${OPENHRP_HOME}/include/idl)

  SET(LIBRARY_NAME openhrp)
  ADD_LIBRARY(${LIBRARY_NAME}
    SHARED
    stack-of-tasks.cc
    stack-of-tasks.hh
    OpenHRPCommon.hh
    OpenHRPCommonSK.cc
    HRPcontroller.h
    HRPcontrollerSK.cc
    ViewSimulator.hh
    ViewSimulatorSK.cc
    )
  ADD_CUSTOM_COMMAND(
    OUTPUT HRPcontroller.h
    COMMAND cp
    ARGS HRPcontroller.hh HRPcontroller.h
    MAIN_DEPENDENCY HRPcontroller.hh
    )

  SET_SOURCE_FILES_PROPERTIES(
    OpenHRPCommon.hh
    OpenHRPCommonSK.cc
    HRPcontroller.hh
    HRPcontroller.h
    HRPcontrollerSK.c
    ViewSimulator.hh
    ViewSimulatorSK.cc
    GENERATED=ON
    )
  SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
  INSTALL(TARGETS ${LIBRARY_NAME}
    DESTINATION lib
    )
  INCLUDE_DIRECTORIES(AFTER
    ${OPENHRP_HOME}/OpenHRP/Controller/corba
    ${OPENHRP_HOME}/include
    ${OPENHRP_HOME}/${HRP_ROBOT_SPEC}
    )
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} "dynamic-graph-corba")
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} "omniORB4")
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} "sot-core")

  DYNAMIC_GRAPH_PYTHON_MODULE("sot/openhrp" ${LIBRARY_NAME} wrap)

  CONFIG_FILES(dynamic_graph/sot/openhrp/prologue.py)
  INSTALL(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/dynamic_graph/sot/openhrp/prologue.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/openhrp/test_reach.py
    DESTINATION
    ${PYTHON_SITELIB}/dynamic_graph/sot/openhrp
    )

  # Corba client
  INSTALL(FILES
    openhrp-shell
    DESTINATION bin
    PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
    )
  SET(SOT_OPENHRP_PLUGIN_INSTALL_DIR "lib")
  IF(INSTALL_IN_OPENHRP)
    SET(SOT_OPENHRP_PLUGIN_INSTALL_DIR
      "${OPENHRP_HOME}/${HRP_ROBOT_SPEC}/bin")
  ENDIF(INSTALL_IN_OPENHRP)

  # Create "stubs" directory before IDL generation
  file(MAKE_DIRECTORY ${${PROJECT_NAME}_BINARY_DIR}/stubs)

  # --- IDL GENERATION ---------------------------------------------------------------------
  INCLUDE(${CMAKE_MODULE_PATH}/AddPluginForOpenHRP.cmake)

  SET(COMMON_IDL_OPENHRP HRPcontroller.idl)
  SET(PLUGIN_NAMESPACE "OpenHRP::")
  SET(DOUBLE_SEQUENCE "dsequence")

  SET(PLUGINBASENAME StackOfTasks)
  SET(PLUGINNAME ${CMAKE_CURRENT_SOURCE_DIR}/${PLUGINBASENAME})
  SET(PLUGINNAME_SRC ${CMAKE_CURRENT_SOURCE_DIR}/plugin.cc)
  SET(PLUGINNAME_IDL "none")
  SET(PLUGINNAME_PATH ${${PROJECT_NAME}_BINARY_DIR}/stubs)
  SET(PLUGINLINKS
    ${BOOST_THREAD_LIB_NAME}
    -pthread)
  SET(PLUGINDEPS)
  SET(PLUGINCOMPILE ${${PROJECT_NAME}_CXX_FLAGS}
    ${OPENHRP_CXX_FLAGS}
    -I${OPENHRP_HOME}/OpenHRP/Controller/IOserver/robot/${HRP_ROBOT_SPEC}
    ${OMNIORB4_CFLAGS}
    ${JRL_MAL_CFLAGS}
    ${SOT_CORE_CFLAGS}
)

  LIST2STRING(PLUGIN_CFLAGS ${PLUGINCOMPILE})
  LIST2STRING(PLUGIN_LDFLAGS ${PLUGINLINKS})

  ADD_OPENHRP_PLUGIN(${PLUGINNAME} ${PLUGINNAME_SRC} ${PLUGINNAME_IDL}
    ${PLUGINNAME_PATH} ${PLUGIN_CFLAGS}
    ${PLUGIN_LDFLAGS} ${LIBRARY_NAME})
  # This will create StackOfTasks.so in ${${PROJECT_NAME}_BINARY_DIR}/lib

  INSTALL(FILES ${${PROJECT_NAME}_BINARY_DIR}/lib/${PLUGINBASENAME}.so
    DESTINATION ${SOT_OPENHRP_PLUGIN_INSTALL_DIR}
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)

ENDIF(OPENHRP_VERSION_3)
