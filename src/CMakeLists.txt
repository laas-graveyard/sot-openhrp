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

  SET(SOT_OPENHRP_PLUGIN_INSTALL_DIR "lib")
  IF(INSTALL_IN_OPENHRP)
    SET(SOT_OPENHRP_PLUGIN_INSTALL_DIR
      "${OPENHRP_HOME}/Controller/IOserver/robot/${HRP_ROBOT_SPEC}/bin")
  ENDIF(INSTALL_IN_OPENHRP)

  # Create "stubs" directory before IDL generation
  file(MAKE_DIRECTORY ${${PROJECT_NAME}_BINARY_DIR}/stubs)

  # --- IDL GENERATION ---------------------------------------------------------------------
  INCLUDE(${CMAKE_MODULE_PATH}/AddPluginForOpenHRP.cmake)

  SET(COMMON_IDL_OPENHRP HRPController.idl)
  SET(PLUGIN_NAMESPACE "OpenHRP::")
  SET(DOUBLE_SEQUENCE "dsequence")

  SET(PLUGINBASENAME StackOfTasks)
  SET(PLUGINNAME ${CMAKE_CURRENT_SOURCE_DIR}/${PLUGINBASENAME})
  SET(PLUGINNAME_SRC ${CMAKE_CURRENT_SOURCE_DIR}/${PLUGINBASENAME}.cpp)
  SET(PLUGINNAME_IDL "none")
  SET(PLUGINNAME_PATH ${${PROJECT_NAME}_BINARY_DIR}/stubs)
  SET(PLUGINLINKS
    ${SOT_CORE_LDFLAGS}
    ${JRL_MAL_LDFLAGS}
    ${BOOST_THREAD_LIB_NAME} -pthread)
  SET(PLUGINDEPS ${OPENHRP_HOME}/Controller/IOserver/plugin/SequencePlayer/corba/SequencePlayer.idl)
  SET(PLUGINCOMPILE ${${PROJECT_NAME}_CXX_FLAGS}
    ${OPENHRP_CXX_FLAGS}
    -I${OPENHRP_HOME}/Controller/IOserver/robot/${HRP_ROBOT_SPEC})

  LIST2STRING(PLUGIN_CFLAGS ${PLUGINCOMPILE})
  LIST2STRING(PLUGIN_LDFLAGS ${PLUGINLINKS})

  ADD_OPENHRP_PLUGIN(${PLUGINNAME} ${PLUGINNAME_SRC} ${PLUGINNAME_IDL}
    ${PLUGINNAME_PATH} ${PLUGIN_CFLAGS}
    ${PLUGIN_LDFLAGS} ${PLUGINDEPS})
  # This will create StackOfTasks.so in ${${PROJECT_NAME}_BINARY_DIR}/lib

  INSTALL(FILES ${${PROJECT_NAME}_BINARY_DIR}/lib/${PLUGINBASENAME}.so
    DESTINATION ${SOT_OPENHRP_PLUGIN_INSTALL_DIR}
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)

ENDIF(OPENHRP_VERSION_3)
