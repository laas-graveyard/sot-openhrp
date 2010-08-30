# Specific to HRP-2
import hrp
import hstsetup
from hrp import *

class Teleop:
  def __init__(self,name):
    self.Name = name 
    # ---------------------------------------------------------------------------- #
    # --- LOADING THE PLUGINS ---------------------------------------------------- #
    # ---------------------------------------------------------------------------- #

    self.ms = findPluginManager("motionsys")
    self.with_trace = 0;

    # --- Kalman filter --- #

    self.ms.load("kfplugin")
    self.kf = self.ms.create("kfplugin","kf","")
    #
    # !!! ZMP sensor is DEPRECATED !!!
    #
    # ms.load("ZMPsensor")
    # zmpsensor = ms.create("ZMPsensor","zmpsensor","")
    #
    self.kf.start()


    # --- Sequence player --- #

    self.ms.load("seqplay")
    self.seq = SequencePlayerHelper.narrow(self.ms.create("seqplay","seq","-ORBconfig../../../../../Common/orb.conf"))
    self.seq.start()


    # --- Stack of Tasks --- #

    #waitInput("Wait before loading SOT plugin.")
    self.ms.load("StackOfTasks")
    self.SoT = self.ms.create("StackOfTasks","SoT","")

    # --- Stabilizer --- #
    self.ms.load("hstabilizer")
    self.st = self.ms.create("hstabilizer","st","")

    # --- Log plugin --- #
    self.ms.load("logplugin")
    self.log = LoggerPluginHelper.narrow(self.ms.create("logplugin","log",""))
    self.log.add("kf")
    self.log.add("st")
    self.log.sendMsg(":max-length 80")
    self.log.start()

    # --- Specific to walking --- #

    hstsetup.stsetup(self.st)
    self.kf.start()
    self.st.start()


    # ---------------------------------------------------------------------------- #
    # --- INITIAL POSITION ------------------------------------------------------- #
    # ---------------------------------------------------------------------------- #

    #seq.goHalfSitting(2.5)
    #INIT TELEOP
    rleg_a = "0 0 -26 50 -24 0"
    lleg_a = "0 0 -26 50 -24 0"
    chest_a = "0 -3"
    head_a = "0 0"

    # Half sitting 
    rarm_a = "15 -10 0 -30 0 0 10"
    larm_a = "15  10 0 -30 0 0 10"
  
    # New Germany
    # rarm_a = "33 -10 0 -120 -10 0 10" # <-- ballrose: "10 -10 -10 -80 0 -0 0", dunnowhat: "10 -10 -10 -70 0 -0 10"
    # larm_a = "33  10 0 -120  10 0 10" # <-- ballrose: "35 5 0 -20.0 0 0 10.0", dunnowhat: "10  10 10 -70 0 0 10"
   
    # Old Germany
    # rarm_a = "10 -18 0 -100 -18 0 10"
    # larm_a = "10  18 0 -100  18 0 10"
    rhand_a = "-10.0 10.0 -10.0 10.0 -10.0"
    lhand_a = "-10.0 10.0 -10.0 10.0 -10.0"
    self.seq.sendMsg(":joint-angles "+rleg_a+" "+lleg_a+" "+chest_a+" "+head_a+" "+rarm_a+" "+larm_a+" "+rhand_a+" "+lhand_a+" 2.5")
    self.seq.waitInterpolation()
    #seq.stop()
    #ms.sendMsg(":destroy seq")
    #ms.sendMsg(":unload seqplay")


    # ---------------------------------------------------------------------------- #
    # --- SOT INITIALIZATION ----------------------------------------------------- #
    # ---------------------------------------------------------------------------- #

    self.SoT.sendMsg(":init")

    #
    # !!! teleop runs with dynsmall, NOT with standard dyn !!!
    #
    self.SoT.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/dynsmall")
    self.SoT.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/coshell")
    self.SoT.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/traces")

    self.SoT.sendMsg(":script plug OpenHRP.state dyn.position")
    self.SoT.sendMsg(":script plug OpenHRP.state dyn2.position")
    self.SoT.sendMsg(":script plug OpenHRP.attitude posKF.attitudeIN")
    self.SoT.sendMsg(":script plug OpenHRP.attitude flex.sensorWorldRotation")
    self.SoT.sendMsg(":script plug sot.control OpenHRP.control")

    self.SoT.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/force")
    self.SoT.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/forceL")
    self.SoT.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/small")
    self.SoT.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/jointlimit")
    self.SoT.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/grip")

    self.SoT.sendMsg(":script OpenHRP.pause")
    self.SoT.start()
    self.SoT.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/hwpg")

    self.SoT.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/teleoperation/teleop")
    self.SoT.sendMsg(":script sot.clear")

  def Play(self):
    # --- Play --- #
    #waitInputConfirm("Launch the teleop client then click [OK] to start.")

    self.SoT.sendMsg(":script OpenHRP.reinit from mc")
    self.SoT.sendMsg(":script OpenHRP.play")
    
    if(self.with_trace):
      self.SoT.sendMsg(":script tr.start")
    waitInputConfirm("click [OK] to continue.")

  def LaunchWalking(self):
    # --- Walking --- #
    # 

    self.SoT.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/hwpg")
    self.SoT.sendMsg(":script run ${CMAKE_INSTALL_PREFIX}/script/walking/hwpgjointsbscw")
    waitInputConfirm("Click [OK] to continue.")

  def StopAndCleanup(self):
    # ---------------------------------------------------------------------------- #
    # --- STOP AND CLEANUP ------------------------------------------------------- #
    # ---------------------------------------------------------------------------- #
    # -------------------------------------------------------------------------
    # waitInputConfirm("Click [OK] to stop the SoT.")
    self.SoT.sendMsg(":hold")
    #
    # Careful! The SOT plugin should not be stopped before seqplay has ended its
    # motion. waitInterpolation should block the script. If not, don't push [OK] to
    # the next dialog box before the robot has reached the 1/2seating
    #
    if(self.with_trace):
      self.SoT.sendMsg(":script tr.trace")
    self.SoT.sendMsg(":waitForFinished")

    # -------------------------------------------------------------------------
    self.SoT.stop()

    # -------------------------------------------------------------------------
    self.SoT.sendMsg(':profile')
    self.ms.sendMsg(":destroy SoT")
    self.ms.sendMsg(":unload StackOfTasks")

    # --- 
    # --- LOGS
    # --- 
    # waitInputConfirm("Click [OK] to log")
    self.log.stop()
    self.log.save("TeleopNoneModal")

    print("Script finished")


TeleopWithMunich = Teleop("MunichJRL")

defaultMenuE = [[
      '------- Sequence ----------', '#label',
      '1. Play',                     'TeleopWithMunich.Play()',
      '2. LaunchWalking',            'TeleopWithMunich.LaunchWalking()',
      '3. StopAndCleanup',           'TeleopWithMunich.StopAndCleanup()'
    ]]

waitInputMenu(defaultMenuE)
