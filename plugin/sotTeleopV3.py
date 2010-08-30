# Specific to HRP-2
import hrp
import hstsetup
from hrp import *


# ---------------------------------------------------------------------------- #
# --- LOADING THE PLUGINS ---------------------------------------------------- #
# ---------------------------------------------------------------------------- #

ms = findPluginManager("motionsys")


# --- Kalman filter --- #

ms.load("kfplugin")
kf = ms.create("kfplugin","kf","")
kf.start()


# --- Sequence player --- #

ms.load("seqplay")
seq = SequencePlayerHelper.narrow(ms.create("seqplay","seq","-ORBconfig../../../../../Common/orb.conf"))
seq.start()


# --- Stack of Tasks --- #

ms.load("StackOfTasks")
SoT = ms.create("StackOfTasks","SoT","")


# --- Stabilizer --- #

ms.load("hstabilizer")
st = ms.create("hstabilizer","st","")


# --- Log plugin --- #

ms.load("logplugin")
log = LoggerPluginHelper.narrow(ms.create("logplugin","log",""))
log.add("kf")
log.add("st")
log.sendMsg(":max-length 80")
log.start()


# --- Specific to walking --- #

hstsetup.stsetup(st)
kf.start()
st.start()


# ---------------------------------------------------------------------------- #
# --- INITIAL POSITION ------------------------------------------------------- #
# ---------------------------------------------------------------------------- #

#seq.goHalfSitting(2.5)
rleg_a = "0 0 -26 50 -24 0"
lleg_a = "0 0 -26 50 -24 0"
chest_a = "0 0"
head_a = "0 0"
rarm_a = "10 -18 0 -100 -18 0 10" # <-- ballrose: "10 -10 -10 -80 0 -0 0", dunnowhat: "10 -10 -10 -70 0 -0 10"
larm_a = "10  18 0 -100  18 0 10" # <-- ballrose: "35 5 0 -20.0 0 0 10.0", dunnowhat: "10  10 10 -70 0 0 10"
rhand_a = "-10.0 10.0 -10.0 10.0 -10.0"
lhand_a = "-10.0 10.0 -10.0 10.0 -10.0"
seq.sendMsg(":joint-angles "+rleg_a+" "+lleg_a+" "+chest_a+" "+head_a+" "+rarm_a+" "+larm_a+" "+rhand_a+" "+lhand_a+" 2.5")
seq.waitInterpolation()

# ---------------------------------------------------------------------------- #
# --- SOT INITIALIZATION ----------------------------------------------------- #
# ---------------------------------------------------------------------------- #

SoT.sendMsg(":init")

SoT.sendMsg(":script run /home/blue/sot-lib/script/dynsmall")
SoT.sendMsg(":script run /home/blue/sot-lib/script/coshell")
SoT.sendMsg(":script run /home/blue/sot-lib/script/traces")

SoT.sendMsg(":script plug OpenHRP.state dyn.position")
SoT.sendMsg(":script plug OpenHRP.state dyn2.position")
SoT.sendMsg(":script plug OpenHRP.attitude posKF.attitudeIN")
SoT.sendMsg(":script plug OpenHRP.attitude flex.sensorWorldRotation")
SoT.sendMsg(":script plug sot.control OpenHRP.control")

SoT.sendMsg(":script run /home/blue/sot-lib/script/force")
SoT.sendMsg(":script run /home/blue/sot-lib/script/forceL")
SoT.sendMsg(":script run /home/blue/sot-lib/script/small")
SoT.sendMsg(":script run /home/blue/sot-lib/script/jointlimit")
#
# SoT.sendMsg(":script run /home/blue/sot-lib/script/collisiondetection")
#
SoT.sendMsg(":script run /home/blue/sot-lib/script/grip")

SoT.sendMsg(":script OpenHRP.pause")
SoT.start()

# This dialog box to make sure that OpenHRP.state is really initialized 
# before going on.
waitInputConfirm("Chotto matte kudasai ne (click [OK] to continue).")

SoT.sendMsg(":script run /home/evrard/devel/openrobots/script/hwpgpginit")
SoT.sendMsg(":script run /home/evrard/devel/openrobots/script/hwpginitframes")
SoT.sendMsg(":script run /home/blue/sot-lib/script/dynmc")
SoT.sendMsg(":script run /home/blue/sot-lib/script/teleoperation/teleopAll")
SoT.sendMsg(":script sot.clear")

# --- Play --- #

waitInputConfirm("Launch the teleop client then click [OK] to start.")

SoT.sendMsg(":script OpenHRP.reinit from mc")
SoT.sendMsg(":script OpenHRP.play")
with_trace = 0
if(with_trace):
      SoT.sendMsg(":script tr.start")

# --- Walking --- #

waitInputConfirm("Click [OK] to launch walking.")

SoT.sendMsg(":script run /home/blue/sot-lib/script/walking/hwpgjointsteleop")
SoT.sendMsg(":script run /home/blue/sot-lib/script/walkreact")


# ---------------------------------------------------------------------------- #
# --- STOP AND CLEANUP ------------------------------------------------------- #
# ---------------------------------------------------------------------------- #

# -------------------------------------------------------------------------
waitInputConfirm("Click [OK] to stop the SoT.")
SoT.sendMsg(":hold")
#
# Careful! The SOT plugin should not be stopped before seqplay has ended its
# motion. waitInterpolation should block the script. If not, don't push [OK] to
# the next dialog box before the robot has reached the 1/2seating
#
if(with_trace):
      SoT.sendMsg(":script tr.trace")
SoT.sendMsg(":waitForFinished")

# -------------------------------------------------------------------------
SoT.stop()

# -------------------------------------------------------------------------
SoT.sendMsg(':profile')
ms.sendMsg(":destroy SoT")
ms.sendMsg(":unload StackOfTasks")

# --- 
# --- LOGS
# --- 
waitInputConfirm("Click [OK] to log")
log.stop()
log.save("teleop")

print("Script finished")


