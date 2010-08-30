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
			 WORKINGDIRIDL PLUGINCOMPILE PLUGINLINKS CORBAPLUGINDEPS)

GET_FILENAME_COMPONENT(PluginBaseName ${PLUGINNAME} NAME)
GET_FILENAME_COMPONENT(PluginBasePath ${PLUGINNAME} PATH)
GET_FILENAME_COMPONENT(IDLBaseName ${PLUGINCORBAIDL} NAME)

# Collect all the sources (mainplugin cpp plus idl dependancies).
SET(PLUGIN_SRCS ${PLUGINNAME_SRCS})
IF(NOT ${PLUGINNAME_SRCS})
  SET(PLUGINNAME_SRCS "${PluginBasePath}/${PluginBaseName}.cpp")
ENDIF(NOT ${PLUGINNAME_SRCS})

# --- IDLS -----------------------------------------------------------------
# --- IDLS -----------------------------------------------------------------
# --- IDLS -----------------------------------------------------------------

# Set the directories for idl dependancies.
SET(IDL_INCLUDE_DIR "${OPENHRP_HOME}/Common/corba")
IF (OPENHRP_VERSION_3)
  LIST(APPEND IDL_INCLUDE_DIR ${OPENHRP_HOME}/DynamicsSimulator/corba)
  LIST(APPEND IDL_INCLUDE_DIR ${OPENHRP_HOME}/Controller/IOserver/corba)
  LIST(APPEND IDL_INCLUDE_DIR ${OPENHRP_HOME}/Controller/IOserver/plugin/SequencePlayer/corba)
  LIST(APPEND IDL_INCLUDE_DIR ${OPENHRP_HOME}/CollisionDetector/corba/)
  LIST(APPEND IDL_INCLUDE_DIR ${OPENHRP_HOME}/ViewSimulator/corba/)
  LIST(APPEND IDL_INCLUDE_DIR ${OPENHRP_HOME}/ModelLoader/corba/)
ENDIF(OPENHRP_VERSION_3)

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
SET(IDL_FILES_FOR_OPENHRP ${CORBAPLUGINDEPS})
IF (OPENHRP_VERSION_2)
  LIST(APPEND IDL_FILES_OPENHRP ${OPENHRP_HOME}/Common/corba/common.idl)
ELSE(OPENHRP_VERSION_2) # openhrp_version_3
  LIST(APPEND IDL_FILES_FOR_OPENHRP 
      ${OPENHRP_HOME}/Common/corba/OpenHRPCommon.idl
      ${OPENHRP_HOME}/Controller/IOserver/corba/HRPcontroller.idl
      ${OPENHRP_HOME}/ViewSimulator/corba/ViewSimulator.idl
      ${OPENHRP_HOME}/DynamicsSimulator/corba/DynamicsSimulator.idl
      ${OPENHRP_HOME}/ModelLoader/corba/ModelLoader.idl	
      ${OPENHRP_HOME}/CollisionDetector/corba/CollisionDetector.idl	
   )
ENDIF(OPENHRP_VERSION_2)
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
ADD_LIBRARY(${PluginBaseName} ${PLUGIN_SRCS})

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
       -I${OPENHRP_HOME}/Controller/IOserver/include -I${OPENHRP_HOME}/Common
       -I${OPENHRP_HOME}/Controller/common -I${OPENHRP_HOME}/Controller/IOserver/sys/plugin
       -I${WORKINGDIRIDL} -pthread ${${PROJECT_NAME}_CXX_FLAGS} ${omniORB4_cflags})
IF (OPENHRP_VERSION_2)
  LIST(APPEND PLUGIN_CFLAGS -DOPENHRP_VERSION_2)
ELSE(OPENHRP_VERSION_2) # OpenHRP3
  LIST(APPEND PLUGIN_CFLAGS -DOPENHRP_VERSION_3 -I${OPENHRP_HOME}/DynamicsSimulator/server/)
ENDIF (OPENHRP_VERSION_2)
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
