##############################################################################
#
# Copyright JRL, CNRS/AIST, 2008
# 
# Description:
# Try to find OpenHRP/Make.Rules
# capabilities.
# Once run this will define: 
#
# OpenHRP_FOUND
# OpenHRP_DIR
#
# Authors:
# Olivier Stasse
#
#############################################################################
SET(HAVE_OPENHRP 0)
SET(OPENHRP_FOUND FALSE)
SET(OPENHRP_VERSION_2 0)
SET(OPENHRP_VERSION_3 0)

# --- FIND ---
# Try to locate OpenHRP-2 and OpenHRP-3.
FIND_PATH(OPENHRP_HOME NAMES Controller/IOserver/include/plugin.h
    		       PATHS $ENV{OPENHRPHOME}
    		       $ENV{HOME}/src/OpenHRP )
FIND_PATH(OPENHRP3_HOME NAME include/idl/HRPcontroller.idl
    			PATHS $ENV{OPENHRPHOME}
    			      $ENV{HOME}/src/OpenHRP $ENV{HOME}/src/OpenHRP-3
			      $ENV{HOME} )

# --- MACRO ---
# Define automatic variables for the robot model, depending on the OH version.
MACRO(OPENHRP_CONFIG_DIR_DEFAULT MODEL_DIR CONFIG_DIR)
    # Default value for HRP2_MODEL_DIRECTORY.
    IF(NOT HRP2_MODEL_DIRECTORY)
      UNSET(HRP2_MODEL_DIRECTORY CACHE)
      SET(HRP2_MODEL_DIRECTORY ${MODEL_DIR}
      			       CACHE PATH "Where to find HRP2main.wrl")
    ENDIF(NOT HRP2_MODEL_DIRECTORY)
    # Default value for HRP2_CONFIG_DIRECTORY.
    IF(NOT HRP2_CONFIG_DIRECTORY)
      UNSET(HRP2_CONFIG_DIRECTORY CACHE)
      SET(HRP2_CONFIG_DIRECTORY ${CONFIG_DIR}
      				CACHE PATH "Where to find HRP2Specificities.xml")
    ENDIF(NOT HRP2_CONFIG_DIRECTORY)
ENDMACRO(OPENHRP_CONFIG_DIR_DEFAULT)

# --- DEFAULT VARS --- 
# Initialize variables depending on OH version.
IF(OPENHRP_HOME OR OPENHRP3_HOME)
  SET(OPENHRP_FOUND TRUE)
  SET(HAVE_OPENHRP 1)

  IF(OPENHRP3_HOME)
    SET(OPENHRP_VERSION_3 1)
    SET(OPENHRP_HOME ${OPENHRP3_HOME})  
    OPENHRP_CONFIG_DIR_DEFAULT(${OPENHRP_HOME}/HRP2JRL/model/ 
                               ${OPENHRP_HOME}/HRP2JRL/etc/)
  ELSE(OPENHRP3_HOME)
    SET(OPENHRP_VERSION_2 1)
    OPENHRP_CONFIG_DIR_DEFAULT(${OPENHRP_HOME}/etc/HRP2JRL/
			       ${OPENHRP_HOME}/Controller/IOserver/robot/HRP2JRL/bin/)
  ENDIF(OPENHRP3_HOME)

  IF(OMNIORB4_FOUND)
    SET(OPENHRP_CXX_FLAGS "-DCOMMERCIAL -Wall -Wunused ")
    SET(OPENHRP_LDD_FLAGS "")
  ENDIF(OMNIORB4_FOUND)

  # Write version of OpenHRP in terminal
  IF(OPENHRP2_HOME)
    MESSAGE(STATUS "OpenHRP 2: ${OPENHRP_HOME}")
  ENDIF(OPENHRP2_HOME)
  IF(OPENHRP3_HOME)
    MESSAGE(STATUS "OpenHRP 3: ${OPENHRP_HOME}")
  ENDIF(OPENHRP3_HOME)

ELSE(OPENHRP_HOME OR OPENHRP3_HOME)
  SET(OPENHRP_FOUND FALSE)
  MESSAGE(STATUS "OpenHRP not found" )
ENDIF(OPENHRP_HOME OR OPENHRP3_HOME)

#SET(${openhrp_final_plugin_path} "${OPENHRP_HOME}/Controller/IOserver/robot/${ROBOT}/bin")


