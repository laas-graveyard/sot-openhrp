# Specific to HRP-2
import hrp
import hstsetup
from hrp import *

# ---------------------------------------------------------------------------- #
# --- LOADING THE PLUGINS ---------------------------------------------------- #
# ---------------------------------------------------------------------------- #

ms = findPluginManager("motionsys")


# ---
# --- CLASSICAL PLUGIN: filter, ZMP, Seq.
# ---
ms.load("kfplugin")
kf = ms.create("kfplugin","kf","")
#ms.load("ZMPsensor")
#zmpsensor = ms.create("ZMPsensor","zmpsensor","")
kf.start()

ms.load("seqplay")
seq = ${SEQPLAYERTYPENAME}Helper.narrow(ms.create("seqplay","seq","-ORBconfig../../../../../Common/orb.conf"))
seq.start()
seq.goHalfSitting(2.5)
seq.waitInterpolation()

# ---
# --- Stack Of Tasks
# ---
#waitInput("Wait before loading SOT plugin.");
ms.load("StackOfTasks")
SoT = ms.create("StackOfTasks","SoT","")

# ---
# --- STABILIZER
# ---
ms.load("hstabilizer")
st = ms.create("hstabilizer","st","")

# ---
# --- LOG
# ---
ms.load("logplugin")
log = LoggerPluginHelper.narrow(ms.create("logplugin","log",""))
log.add("kf")
log.add("st")
log.sendMsg(":max-length 80")
log.start()

# --- 
# --- SPECIFIC TO WALKING
# --- 
hstsetup.stsetup(st)
kf.start()
st.start()

# -------------------------------------------------------------------------
#waitInputConfirm("Wait before SoT init.")
SoT.sendMsg(":init");
#SoT.start()
#SoT.stop()

#waitInputConfirm("Wait before starting.")

#SoT.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/debug")
SoT.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/dyn")
SoT.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/coshell")
SoT.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/traces")

#SoT.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/unittests/ik_arm")
#SoT.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/flexibility")


# --- Use/ignore force sensors
#SoT.sendMsg(":setForces 0 1")
#SoT.sendMsg(":setForces 1 1")
SoT.sendMsg(":setForces 2 1")
#SoT.sendMsg(":setForces 3 1")

SoT.sendMsg(":script plug OpenHRP.state dyn.position")
SoT.sendMsg(":script plug OpenHRP.state dyn2.position")
SoT.sendMsg(":script plug OpenHRP.attitude posKF.attitudeIN")
SoT.sendMsg(":script plug OpenHRP.attitude flex.sensorWorldRotation")
SoT.sendMsg(":script plug sot.control OpenHRP.control")
#SoT.sendMsg(":script plug OpenHRP.forceRLEG ...")
#SoT.sendMsg(":script plug OpenHRP.forceLLEG ...")
#SoT.sendMsg(":script plug OpenHRP.forceRARM ...")
#SoT.sendMsg(":script plug OpenHRP.forceLLEG ...")

SoT.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/jointlimit")

SoT.sendMsg(":script OpenHRP.pause")
SoT.start()

waitInputConfirm("Wait before SoT start.")
SoT.sendMsg(":script OpenHRP.play")



# -------------------------------------------------------------------------


waitInputConfirm("Wait before SoT stop.")
SoT.sendMsg(":hold")
# Careful! The SOT plugin should not be stopped before seqplay has ended its
# motion. waitInterpolation should block the script. If not, don't push [OK] to
# the next dialog box before the robot has reached the 1/2seating
SoT.sendMsg(":waitForFinished")

# -------------------------------------------------------------------------
waitInputConfirm("Wait before SoT stop.")
SoT.stop()

# -------------------------------------------------------------------------
waitInputConfirm("Wait before SoT unload.")
SoT.sendMsg(':profile')
ms.sendMsg(":destroy SoT")
ms.sendMsg(":unload StackOfTasks")

waitInputConfirm("Log and script finished ")


# --- 
# --- LOGS
# --- 
#waitInputConfirm("Wait before log")
log.stop()
log.save("sim")

print("Script finished")


