# 
#  Copyright 
# 

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

    SET(HRP_ROBOT_SPEC HRP2JRL CACHE STRING "Robot name, to resolve specific path in OpenHRP.")

    SET(SOT_OPENHRP_SCRIPT_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    SET(SOT_OPENHRP_SCRIPT_BUILD_DIR "${${PROJECT_NAME}_BINARY_DIR}/plugin")
    SET(SOT_OPENHRP_SCRIPT_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/plugin")
    SET(SOT_OPENHRP_PLUGIN_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib")
    IF(INSTALL_IN_OPENHRP)
	SET(SOT_OPENHRP_SCRIPT_INSTALL_DIR "${OPENHRP_HOME}/Controller/IOserver/robot/HRP2JRL/script")
        SET(SOT_OPENHRP_PLUGIN_INSTALL_DIR "${OPENHRP_HOME}/Controller/IOserver/robot/${HRP_ROBOT_SPEC}/bin")
    ENDIF(INSTALL_IN_OPENHRP)

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
    MESSAGE("link flags are ${${PROJECT_NAME}_LD_FLAGS}")
    SET(PLUGINLINKS ${${PROJECT_NAME}_BINARY_DIR}/lib/libsot-core.so 
    ${${PROJECT_NAME}_BINARY_DIR}/lib/libdynamic-graph.so
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
    INSTALL(FILES ${${PROJECT_NAME}_BINARY_DIR}/lib/${PLUGINBASENAME}.so
    		  DESTINATION ${SOT_OPENHRP_PLUGIN_INSTALL_DIR}
		  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)

    # --- SCRIPT GENERATION AND INSTALL -----------------------------------------------------
    SET(SEQPLAYERTYPENAME "SequencePlayer")
    SET(SOT_OPENHRP_SCRIPT_TO_INSTALL 
          TestStackOfTasksV3.py.cmake
          sotTeleopV3.py.cmake
	  sotTeleopV3nonemodal.py.cmake
	  sotGeometry.py.cmake
	  sotTaskLearning.py.cmake
	  sotWalkTask.py.cmake
	  sotSOTH.py.cmake
	  sotWalkTaskGeometry.py.cmake
	  sot_plugin.py.cmake
	  TestHWPGinSOT.py.cmake
	  TestHWPGinSOT.py.cmake
	  demoBSCWreviewmeeting.py.cmake
	  sotWalkTaskDemoROBOTatCWE.py.cmake
          sotTutorial.py.cmake
	  )

    FOREACH(SCRIPT_NAME ${SOT_OPENHRP_SCRIPT_TO_INSTALL})
      SET(cmakeextension "\\.cmake")
      STRING(REGEX REPLACE "${cmakeextension}$" "" TARGET_NAME ${SCRIPT_NAME})
      CONFIGURE_FILE(${SOT_OPENHRP_SCRIPT_SOURCE_DIR}/${SCRIPT_NAME}
   	             ${SOT_OPENHRP_SCRIPT_BUILD_DIR}/${TARGET_NAME})
      INSTALL(FILES ${SOT_OPENHRP_SCRIPT_BUILD_DIR}/${TARGET_NAME}
	      DESTINATION ${SOT_OPENHRP_SCRIPT_INSTALL_DIR}
	      PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)
   ENDFOREACH(SCRIPT_NAME)

  ENDIF(OPENHRP_VERSION_3)














#SET(LIBRARY_NAME ${PROJECT_NAME})

#INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

#ADD_DEFINITIONS(-DDEBUG=2)

# provide path to library libsot-core.so
#LINK_DIRECTORIES(${SOT_CORE_LIBRARY_DIRS})

# Add sot-core compilation flags and link to library libsot-core.so
#ADD_DEFINITIONS(${SOT_CORE_CFLAGS})
IF(UNIX)
#TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${SOT_CORE_LIBRARIES})
ENDIF(UNIX)

IF(WIN32)
foreach(dlink ${SOT_CORE_LDFLAGS})
#	SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
endforeach(dlink)
ENDIF(WIN32)

#SET_TARGET_PROPERTIES(${LIBRARY_NAME}
#  PROPERTIES
#  LINK_FLAGS "${${PROJECT_NAME}_src_LDFLAGS}"
#)
