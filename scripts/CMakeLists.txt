# Olivier Stasse, JRL-CNRS
#          from Nicolas Mansard, LAAS-CNRS
# 	   from Olivier Stasse
# 10/07/2009
# 22/07/2009 Adding installation in the proper directory (OS.)

# Where is the trace repository?
IF(WIN32)
  SET(TRACE_REPOSITORY "C:/tmp/")
ELSEIF(UNIX)
    SET(TRACE_REPOSITORY "/tmp/")
ENDIF(WIN32)

SET(DYN_LIB_EXT ${CMAKE_SHARED_LIBRARY_SUFFIX})

# --- MAIN DIRECTORIES ---------------------------------
SET(cmakeextension "\\.cmake")
SET(${PROJECT_NAME}_SCRIPT_SOURCE_DIR ${${PROJECT_NAME}_SOURCE_DIR}/scripts)
SET(${PROJECT_NAME}_SCRIPT_BUILD_DIR ${${PROJECT_NAME}_BINARY_DIR}/script)
SET(${PROJECT_NAME}_SCRIPT_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/script)

# --- MACROS -------------------------------------------
MACRO(SOT_GENERATE_SCRIPT listofscripttocreate)
  FOREACH(scripttochange ${ARGV})
    STRING(REGEX REPLACE "${cmakeextension}$" "" scripttarget ${scripttochange})
    CONFIGURE_FILE(${${PROJECT_NAME}_SCRIPT_SOURCE_DIR}/${scripttochange}
		   ${${PROJECT_NAME}_SCRIPT_BUILD_DIR}/${scripttarget})

    SET(ISTHEREADIRECTORY "")
    STRING(REGEX MATCH "/" ISTHEREADIRECTORY ${scripttochange})

    SET(SCRIPTDESTINATIONDIR ${${PROJECT_NAME}_SCRIPT_INSTALL_DIR} )
    IF(ISTHEREADIRECTORY)
       STRING(REGEX MATCH "[a-zA-Z0-9]*/" THEDIRECTORY1 ${scripttochange})
       STRING(REGEX REPLACE "/" "" THEDIRECTORY2 ${THEDIRECTORY1})
       SET(SCRIPTDESTINATIONDIR "${${PROJECT_NAME}_SCRIPT_INSTALL_DIR}/${THEDIRECTORY2}" )
    ENDIF(ISTHEREADIRECTORY)

    INSTALL(FILES ${${PROJECT_NAME}_SCRIPT_BUILD_DIR}/${scripttarget}
            DESTINATION ${SCRIPTDESTINATIONDIR}
	    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)
  ENDFOREACH(scripttochange)
ENDMACRO(SOT_GENERATE_SCRIPT)

MACRO(SOT_INSTALL_SCRIPT listofscripttoinstall )


  FOREACH(scripttoinstall ${ARGV})

    SET(ISTHEREADIRECTORY "")
    STRING(REGEX MATCH "/" ISTHEREADIRECTORY ${scripttoinstall})
    SET(SCRIPTDESTINATIONDIR ${${PROJECT_NAME}_SCRIPT_INSTALL_DIR} )

    IF(ISTHEREADIRECTORY)
       STRING(REGEX MATCH "[a-zA-Z0-9]*/" THEDIRECTORY1 ${scripttoinstall})
       STRING(REGEX REPLACE "/" "" THEDIRECTORY2 ${THEDIRECTORY1})
       SET(SCRIPTDESTINATIONDIR "${${PROJECT_NAME}_SCRIPT_INSTALL_DIR}/${THEDIRECTORY2}" )
    ENDIF(ISTHEREADIRECTORY)

    INSTALL(FILES ${${PROJECT_NAME}_SCRIPT_SOURCE_DIR}/${scripttoinstall}
            DESTINATION ${SCRIPTDESTINATIONDIR}
	    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)
  ENDFOREACH(scripttoinstall)
ENDMACRO(SOT_INSTALL_SCRIPT)

# ----------------------------------------------------
# ---- PLUGIN LIST -----------------------------------
# ----------------------------------------------------

SET(PLUGIN_LIST_FILENAME plugin.txt)
#NOTE: plugins list disabled for now, assuming all present
 SET(BoostThread_Plugins "distant-shell${DYN_LIB_EXT}\nperiodic-call-entity${DYN_LIB_EXT}")
 SET(unix_Plugins "mailbox-vector${DYN_LIB_EXT}\n")
 SET(dynamicsJRLJapan_Plugins "dynamic${DYN_LIB_EXT}")
 SET(hrp2Dynamics_Plugins "dynamic-hrp2${DYN_LIB_EXT}")
 SET(walkGenJrl_Plugins "selector${DYN_LIB_EXT}\nwhich-foot-upper${DYN_LIB_EXT}\nnext-step${DYN_LIB_EXT}")
 
# --- Specific plugins list
# Add the plugins for walkGenJrl only if the walkGenJrl is installed
IF (_walkGenJrl_FOUND)
  SET(walkGenJrl_Plugins "selector${DYN_LIB_EXT}\nwhich-foot-upper${DYN_LIB_EXT}\next-step${DYN_LIB_EXT}")
ENDIF(_walkGenJrl_FOUND)
# Add the plugins for jrl-dyn only if installed
IF (_dynamicsJRLJapan_FOUND)
  SET(dynamicsJRLJapan_Plugins "dynamic${DYN_LIB_EXT}")
ENDIF (_dynamicsJRLJapan_FOUND)
# Add the plugins for hrp2-dyn only if installed
IF (_hrp2Dynamics_FOUND)
  SET(hrp2Dynamics_Plugins "dynamic-hrp2${DYN_LIB_EXT}")
ENDIF (_hrp2Dynamics_FOUND)
IF (Boost_THREAD_FOUND)
  SET(BoostThread_Plugins "distant-shell${DYN_LIB_EXT}\nperiodic-call-entity${DYN_LIB_EXT}")
ENDIF (Boost_THREAD_FOUND)
# Add the plugins for Linux only
IF(NOT WIN32)
#    SET(unix_Plugins "mailbox${DYN_LIB_EXT}\n")
ENDIF(NOT WIN32)

# --- GENERATE ---------------------------------------
# ----------------------------------------------------
# Generate the script files dependent on the install.

SET(listofscripttocreate
     ${PLUGIN_LIST_FILENAME}.cmake
     base.cmake
     coshell.cmake
     dyn.cmake
     dynmc.cmake
     dynsmall.cmake
     dynsmallpart1hrp2_10.cmake
     dynsmallpart1hrp2_10_old.cmake
     hwpg.cmake
     hwpgpginit.cmake
     traces.cmake
)

OPTION(GENERATE_DYNFILES "Generate the dyn-files giving the model-file pathes." 1)
IF(GENERATE_DYNFILES)
  LIST(APPEND listofscripttocreate 
    dynfilessmall.cmake 
    dynfilessmallhrp2_10.cmake
    dynfilessmallhrp2_10_old.cmake
    simu.cmake simusmall.cmake)

  # Dynfiles requires HRP2_MODEL_DIRECTORY and HRP2_CONFIG_DIRECTORY
  IF(HRP2_MODEL_DIRECTORY)
    IF(HRP2_CONFIG_DIRECTORY)
	SET (HRP2_FILES_FOUND 1)
    ELSE(HRP2_CONFIG_DIRECTORY)
	SET (HRP2_FILES_FOUND 0)
	MESSAGE (STATUS " HRP2_CONFIG_DIRECTORY not found. Can not create dynfiles")
    ENDIF(HRP2_CONFIG_DIRECTORY)
  ELSE(HRP2_MODEL_DIRECTORY)
    SET (HRP2_FILES_FOUND 0)
    MESSAGE (STATUS " HRP2_MODEL_DIRECTORY not found. Can not create dynfiles")
  ENDIF(HRP2_MODEL_DIRECTORY)

  IF(HRP2_FILES_FOUND)
    LIST(APPEND listofscripttocreate dynfiles.cmake)
  ENDIF(HRP2_FILES_FOUND)
ENDIF(GENERATE_DYNFILES)

# Generate the scripts from script.cmake.
SOT_GENERATE_SCRIPT(${listofscripttocreate})

# --- INSTALL ----------------------------------------
SOT_INSTALL_SCRIPT(acc
		   calib
		   dynsmallpart2
		   force
		   forceL
		   forceSimu
		   grip
		   handsAltitude
		   hwpginitframes
		   jointlimit
		   pgcom
		   selector
		   small
		   smallmc
		   traj
		   walkreact
		   stepping
		   taskPosture
		   clamp-workspace
)

# --- COLLISION --------
SOT_GENERATE_SCRIPT(collisiondetection.cmake)

# --- TASKSEQ OPTIM --------
SOT_GENERATE_SCRIPT(optimization/frigoSmall.cmake)
SOT_INSTALL_SCRIPT(optimization/frigo)

# --- STABILIZER -----------
SOT_INSTALL_SCRIPT(stab/flex
		   stab/flexibility
		   stab/sylvain)

# --- TELEOP ---------------
SOT_INSTALL_SCRIPT(teleoperation/initConnec
		   teleoperation/initConnecPhantom
		   teleoperation/initConnecVFwave
		   teleoperation/initConnecAll
		   teleoperation/teleop
		   teleoperation/teleopVFwave
		   teleoperation/teleopAll
		   teleoperation/data_epfl
		   teleoperation/data_tum)

# --- TLEARN ---------------
SOT_GENERATE_SCRIPT(taskLearning/taskLearning.cmake)

# --- UNITEST --------------
SOT_INSTALL_SCRIPT(unittests/chrono
		   unittests/ik_arm
		   unittests/ik_arm_sensor
		   unittests/ik_arm_sensor_passive
		   unittests/ik_arm_sensor_vel
		   unittests/ik_com
		   unittests/ik_jl_arm)
SOT_GENERATE_SCRIPT(unittests/testInertia.cmake
                    unittests/chronoQr.cmake)

# --- WALK -----------------
SOT_INSTALL_SCRIPT(walking/walk
		   walking/walkreact
		   walkreactrobotatcwe
		   walking/hwpgfeettasks
		   walking/hwpgjoints
		   walking/hwpgjointsbscw
		   walking/hwpgrefs
		   walking/hwpgreact
		   walking/hwpgjointsteleop
		   walking/startherdt
		   walking/teststeps
		   walking/teststeps2
		   walking/teststeps3
		   walking/teststeps3small)
SOT_GENERATE_SCRIPT(walking/hwpgfeettasksrel.cmake)
SOT_GENERATE_SCRIPT(walkreact-new.cmake)
SOT_GENERATE_SCRIPT(walkreact-force.cmake)
SOT_GENERATE_SCRIPT(walkreact-joystick.cmake)
