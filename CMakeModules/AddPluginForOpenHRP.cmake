# Nicolas Mansard, LAAS-CNRS
# 	  From: Olivier Stasse, JRL, CNRS/AIST
# Creation: 02/07/2009
# History:
#
# Copyright CNRS/AIST

# Macro for compiling a plugin OpenHRP
# The macro is non-specific, and could be called from any project. It is
# just using the ${PROJECT_NAME}_CXX_FLAGS and ${PROJECT_NAME}_LD_FLAGS
# for compiling the project.
# The result is a rule for compiling the PLUGINNAME.so library, in BUILD_DIR/lib.
# This lib should then be installed in OHRP controller dir.
# Erratum: ${HRP_ROBOT_SPEC} global var is also used, and should  be removed.

MACRO(ADD_OPENHRP_PLUGIN PLUGINNAME PLUGINNAME_SRCS PLUGINCORBAIDL
			 WORKINGDIRIDL PLUGINCOMPILE PLUGINLINKS
			 COMPILEPLUGINDEPS)

GET_FILENAME_COMPONENT(PluginBaseName ${PLUGINNAME} NAME)
GET_FILENAME_COMPONENT(PluginBasePath ${PLUGINNAME} PATH)

# Collect all the sources (mainplugin cpp plus idl dependencies).
SET(PLUGIN_SRCS ${PLUGINNAME_SRCS})
IF(NOT ${PLUGINNAME_SRCS})
  SET(PLUGINNAME_SRCS "${PluginBasePath}/${PluginBaseName}.cpp")
ENDIF(NOT ${PLUGINNAME_SRCS})

# --- IDLS -----------------------------------------------------------------
# --- IDLS -----------------------------------------------------------------
# --- IDLS -----------------------------------------------------------------

# Set the directories for idl dependancies.
SET(IDL_INCLUDE_DIR "${OPENHRP_HOME}/OpenHRP/Common/corba")

LIST(APPEND IDL_INCLUDE_DIR ${OPENHRP_HOME}/OpenHRP/DynamicsSimulator/corba)
LIST(APPEND IDL_INCLUDE_DIR ${OPENHRP_HOME}/include/idl)
LIST(APPEND IDL_INCLUDE_DIR ${OPENHRP_HOME}/OpenHRP/CollisionDetector/corba)
LIST(APPEND IDL_INCLUDE_DIR ${OPENHRP_HOME}/OpenHRP/ViewSimulator/corba)
LIST(APPEND IDL_INCLUDE_DIR ${OPENHRP_HOME}/OpenHRP/ModelLoader/corba)

# --- MAIN IDL -------------------------------------------------------------
# IDL Generation rule.
# Args verification.
IF(EXISTS "${PLUGINCORBAIDL}")
  SET(plugincorbaidl_CPP "${${PROJECT_NAME}_BINARY_DIR}/stubs}/${PluginBaseNameIDL}SK.cc")
  SET(plugincorbaidl_Header "${${PROJECT_NAME}_BINARY_DIR}/stubs}/${PluginBaseNameIDL}.h")
  IDLFILERULE(${PLUGINCORBAIDL} ${plugincorbaidl_CPP} ${plugincorbaidl_Header}
     				${WORKINGDIRIDL} ${IDL_INCLUDE_DIR})
  LIST(APPEND PLUGIN_SRCS ${plugincorbaidl_CPP})
ENDIF(EXISTS "${PLUGINCORBAIDL}")

# --- OHRP GENERIC IDLS ----------------------------------------------------
LIST(APPEND IDL_FILES_FOR_OPENHRP
  ${OPENHRP_HOME}/OpenHRP/Common/corba/OpenHRPCommon.idl
  ${OPENHRP_HOME}/include/idl/HRPcontroller.idl
  ${OPENHRP_HOME}/include/idl/ViewSimulator.idl
  ${OPENHRP_HOME}/include/idl/DynamicsSimulator.idl
  ${OPENHRP_HOME}/include/idl/ModelLoader.idl
  ${OPENHRP_HOME}/include/idl/CollisionDetector.idl
  )

FOREACH( locidlfile ${IDL_FILES_FOR_OPENHRP})
  GET_FILENAME_COMPONENT(locIDLBaseName ${locidlfile} NAME_WE)
  SET(locIDL_CPP "${WORKINGDIRIDL}/${locIDLBaseName}SK.cc")
  SET(locIDL_Header "${WORKINGDIRIDL}/${locIDLBaseName}.h" )
  IDLFILERULE(${locidlfile}
              ${locIDL_CPP} ${locIDL_Header} ${WORKINGDIRIDL} ${IDL_INCLUDE_DIR})
  LIST(APPEND PLUGIN_SRCS ${locIDL_CPP})
ENDFOREACH(locidlfile)

# --- C++ ------------------------------------------------------------------
# --- C++ ------------------------------------------------------------------
# --- C++ ------------------------------------------------------------------

# --- C++ LIB GENERATION ---------------------------------------------------
ADD_LIBRARY(${PluginBaseName} SHARED ${PLUGIN_SRCS})
TARGET_LINK_LIBRARIES(${PluginBaseName} ${COMPILEPLUGINDEPS})
# --- TVMET DEPENDS ??? ---
#FIND_PROGRAM(TVMET_CONFIG_EXECUTABLE NAMES tvmet-config PATHS /usr/local/)
#execute_process(
#    COMMAND ${TVMET_CONFIG_EXECUTABLE} --includes
#    OUTPUT_VARIABLE _tvmet_invoke_result_cxxflags
#    RESULT_VARIABLE _tvmet_failed)
#execute_process(
#    COMMAND ${TVMET_CONFIG_EXECUTABLE} --libs
#    OUTPUT_VARIABLE _tvmet_invoke_result_ldflags
#    RESULT_VARIABLE _tvmet_failed)
#MESSAGE(STATUS "tvmet_cxxflags; ${_tvmet_invoke_result_cxxflags}")
#MESSAGE(STATUS "tvmet_ldflags; ${_tvmet_invoke_result_ldflags}")

# --- C++ FLAGS ------------------------------------------------------------
SET(PLUGIN_CFLAGS ${PLUGINCOMPILE}
  -I${OPENHRP_HOME}/OpenHRP/Common
  -I${WORKINGDIRIDL} -pthread ${${PROJECT_NAME}_CXX_FLAGS} ${omniORB4_cflags})
LIST(APPEND PLUGIN_CFLAGS -DOPENHRP_VERSION_3 -I${OPENHRP_HOME}/OpenHRP/DynamicsSimulator/server/)
#LIST(APPEND PLUGIN_CFLAGS ${_tvmet_invoke_result_cxxflags})

LIST2STRING(_cf ${PLUGIN_CFLAGS})
LIST2STRING(_lf ${omniORB4_link_FLAGS} ${PLUGINLINKS} ${${PROJECT_NAME}_LINK_FLAGS})
SET_TARGET_PROPERTIES(${PluginBaseName}
			PROPERTIES
		        COMPILE_FLAGS ${_cf}
			LINK_FLAGS ${_lf}
			PREFIX "" SUFFIX ".so"
			LIBRARY_OUTPUT_DIRECTORY ${${PROJECT_NAME}_BINARY_DIR}/lib
			)
ENDMACRO(ADD_OPENHRP_PLUGIN)
