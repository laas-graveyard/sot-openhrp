        
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
#seq.waitInterpolation()
#seq.sendMsg(":joint-angles 0 0 -20 40 -20 0                                                                          0 0 -20 40 -20 0                                                                          0 10                                                                                     -0 0                                                                                       10 -10 -10 -50 0 -0 0                                                                     35 5 0 -20.0 0 0 10.0                                                                    -10.0 10.0 -10.0 10.0 -10.0 -10.0 10.0 -10.0 10.0 -10.0  2")
#seq.sendMsg(":joint-angles 0 0 0 0 3 0                                                                               0 0 0 0 3 0                                                                               0 90                                                                                     -0 0                                                                                       10 -10 -10 -50 0 -0 0                                                                     35 5 0 -20.0 0 0 10.0                                                                    -10.0 10.0 -10.0 10.0 -10.0 -10.0 10.0 -10.0 10.0 -10.0  2")
#seq.sendMsg(":joint-angles 0 0 -60 120 -60 0                                                                         0 0 -60 120 -60 0                                                                         0 10                                                                                     -0 0                                                                                       10 -10 -10 -50 0 -0 0                                                                     35 5 0 -20.0 0 0 10.0                                                                    -10.0 10.0 -10.0 10.0 -10.0 -10.0 10.0 -10.0 10.0 -10.0 3")
seq.waitInterpolation()

#ms.sendMsg(":destroy seq")
# ms.sendMsg(":unload seqplay")





# ---
# --- VISUAL SERVOING
# ---
#waitInput("Wait before loading VS plugin.");
ms.load("StackOfTasks")
vs = ms.create("StackOfTasks","vs","")

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

# -------------------------------------------------------------------------
#waitInputConfirm("Wait before vs init.")
vs.sendMsg(":init");
#vs.start()
#vs.stop()

#waitInputConfirm("Wait before starting.")

#vs.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/debug")
vs.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/dyn")
vs.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/coshell")
vs.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/traces")

vs.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/unittests/ik_arm")
vs.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/debug")
#vs.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/flexibility")


# --- Use/ignore force sensors
#vs.sendMsg(":setForces 0 1")
#vs.sendMsg(":setForces 1 1")
vs.sendMsg(":setForces 2 1")
#vs.sendMsg(":setForces 3 1")

vs.sendMsg(":script plug OpenHRP.state dyn.position")
vs.sendMsg(":script plug OpenHRP.state dyn2.position")
vs.sendMsg(":script plug OpenHRP.attitude posKF.attitudeIN")
vs.sendMsg(":script plug sot.control OpenHRP.control")
#vs.sendMsg(":script plug OpenHRP.forceRLEG ...")
#vs.sendMsg(":script plug OpenHRP.forceLLEG ...")
#vs.sendMsg(":script plug OpenHRP.forceRARM ...")
#vs.sendMsg(":script plug OpenHRP.forceLLEG ...")

vs.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/jointlimit")

vs.sendMsg(":script OpenHRP.pause")
vs.start()

waitInputConfirm("Wait before vs start.")
vs.sendMsg(":script OpenHRP.play")



# -------------------------------------------------------------------------


waitInputConfirm("Wait before vs stop.")
vs.sendMsg(":hold")
# Careful! The VS plugin should not be stopped before seqplay has ended its
# motion. waitInterpolation should block the script. If not, don't push [OK] to
# the next dialog box before the robot has reached the 1/2seating
vs.sendMsg(":waitForFinished")

# -------------------------------------------------------------------------
waitInputConfirm("Wait before vs stop.")
vs.stop()

# -------------------------------------------------------------------------
waitInputConfirm("Wait before vs unload.")
vs.sendMsg(':profile')
ms.sendMsg(":destroy vs")
ms.sendMsg(":unload StackOfTasks")

waitInputConfirm("Log and script finished ")


# --- 
# --- LOGS
# --- 
#waitInputConfirm("Wait before log")
log.stop()
log.save("sim")

print("Script finished")


