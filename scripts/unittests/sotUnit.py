        
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
seq = seqpluginHelper.narrow(ms.create("seqplay","seq","-ORBconfig../../../../../Common/orb.conf"))
seq.start()

# ---
# --- VISUAL SERVOING
# ---
#waitInput("Wait before loading VS plugin.");
ms.load("StackOfTasks")
vs = ms.create("StackOfTasks","vs","")

# ---
# --- STABILIZER
# ---
#ms.load("hstabilizer")
#st = ms.create("hstabilizer","st","")

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
#seq.goHalfSitting(2.5)
#BALLROSE seq.sendMsg(":joint-angles 0 0 -20 40 -20 0                                                                          0 0 -20 40 -20 0                                                                          0 0                                                                                      -0 0                                                                                       10 -10 -10 -80 0 -0 0                                                                     35 5 0 -20.0 0 0 10.0                                                                    -10.0 10.0 -10.0 10.0 -10.0 -10.0 10.0 -10.0 10.0 -10.0  2.5")
seq.sendMsg(":joint-angles 0 0 -20 40 -20 0                                                                          0 0 -20 40 -20 0                                                                          0 0                                                                                      -0 0                                                                                       10 -10 -10 -70 0 -0 10                                                                    10  10 10 -70 0 0 10                                                                    -10.0 10.0 -10.0 10.0 -10.0 -10.0 10.0 -10.0 10.0 -10.0  2.5")

seq.waitInterpolation()
#seq.stop()
#ms.sendMsg(":destroy seq")
#ms.sendMsg(":unload seqplay")



# -------------------------------------------------------------------------
#waitInputConfirm("Wait before vs init.")
vs.sendMsg(":init");

#waitInputConfirm("Wait before script loading.")

vs.sendMsg(":script run /home/nmansard/src//StackOfTasks/script/dyn")
vs.sendMsg(":script run /home/nmansard/src//StackOfTasks/script/coshell")
vs.sendMsg(":script run /home/nmansard/src//StackOfTasks/script/traces")


#vs.sendMsg(":script run /home/nmansard/src//StackOfTasks/script/unittests/ik_arm")
#vs.sendMsg(":script run /home/nmansard/src//StackOfTasks/script/unittests/ik_jl_arm")
#vs.sendMsg(":script run /home/nmansard/src//StackOfTasks/script/unittests/ik_arm_sensor_passive")
#vs.sendMsg(":script run /home/nmansard/src//StackOfTasks/script/unittests/ik_arm_sensor")
#vs.sendMsg(":script run /home/nmansard/src//StackOfTasks/script/unittests/ik_arm_sensor_vel")
#vs.sendMsg(":script run /home/nmansard/src//StackOfTasks/script/unittests/ik_com")
#vs.sendMsg(":script run /home/nmansard/src//StackOfTasks/script/unittests/ik_com_sensor")
#vs.sendMsg(":script run /home/nmansard/src//StackOfTasks/script/flexibility")

# --- Use/ignore force sensors
#vs.sendMsg(":setForces 0 1")
#vs.sendMsg(":setForces 1 1")
#vs.sendMsg(":setForces 2 1")
#vs.sendMsg(":setForces 3 1")

vs.sendMsg(":script plug OpenHRP.state dyn.position")
vs.sendMsg(":script plug OpenHRP.state dyn2.position")
vs.sendMsg(":script plug OpenHRP.attitude posKF.attitudeIN")
vs.sendMsg(":script plug OpenHRP.attitude attitudeSensor.in")
vs.sendMsg(":script plug sot.control OpenHRP.control")
vs.sendMsg(":script plug OpenHRP.attitude attitudeSensor.in")
#vs.sendMsg(":script plug OpenHRP.forceRLEG ...")
#vs.sendMsg(":script plug OpenHRP.forceLLEG ...")
#vs.sendMsg(":script plug OpenHRP.forceRARM ...")
#vs.sendMsg(":script plug OpenHRP.forceLLEG ...")

vs.sendMsg(":script OpenHRP.pause")

#waitInputConfirm("Wait before vs start.")
vs.start()

with_trace=waitInputSelect("Wait before vs play.\nPress [Yes] to start the tracer by the way [No] to simply start. ")
vs.sendMsg(":script OpenHRP.reinit from mc")
vs.sendMsg(":script OpenHRP.play")
if(with_trace):
      vs.sendMsg(":script tr.start")

# -------------------------------------------------------------------------
with_trace=waitInputSelect("Wait before vs stop.\nPress [Yes] to force the trace by the way [No] to simply stop.")
vs.sendMsg(":hold")
# Careful! The VS plugin should not be stopped before seqplay has ended its
# motion. waitInterpolation should block the script. If not, don't push [OK] to
# the next dialog box before the robot has reached the 1/2seating
if(with_trace):
      vs.sendMsg(":script tr.trace")
vs.sendMsg(":waitForFinished")

# -------------------------------------------------------------------------
#waitInputConfirm("Wait before vs stop.")
vs.stop()

# -------------------------------------------------------------------------
#waitInputConfirm("Wait before vs unload.")
vs.sendMsg(':profile')
ms.sendMsg(":destroy vs")
ms.sendMsg(":unload StackOfTasks")

# --- 
# --- LOGS
# --- 
waitInputConfirm("Wait before log")
log.stop()
log.save("sim")

print("Script finished")


