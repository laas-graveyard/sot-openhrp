# Specific to HRP-2
import hrp
import hstsetup
from hrp import *

#
# class StackOfTasksDialog
# 
# Non modal dialog box with basic control buttons to initialize
# the SoT framework and link it with OpenHRP.
# 
class StackOfTasksDialog:

  #
  # Constructor
  #
  # - Loads the OpenHRP plugins
  # - Goes to initial position
  # - Initializes the basic components of the SoT
  #
  def __init__(self,name):
    self.Name = name ;
    self.with_trace = 0;

    self.LoadOpenHRPPlugins();
    self.GoInitialPosition();
    self.InitializeSoT();


  #
  # Load OpenHRP plugins
  #
  def LoadOpenHRPPlugins(self):

    self.ms = findPluginManager("motionsys")

    # Kalman filter
    self.ms.load("kfplugin")
    self.kf = self.ms.create("kfplugin","kf","")
    self.kf.start()

    # Sequence player
    self.ms.load("seqplay")
    self.seq = SequencePlayerHelper.narrow(self.ms.create("seqplay","seq","-ORBconfig../../../../../Common/orb.conf"))
    self.seq.start()

    # Stack of Tasks
    self.ms.load("StackOfTasks")
    self.SoT = self.ms.create("StackOfTasks","SoT","")

    # Stabilizer
    self.ms.load("hstabilizer")
    self.st = self.ms.create("hstabilizer","st","")

    # Log plugin
    self.ms.load("logplugin")
    self.log = LoggerPluginHelper.narrow(self.ms.create("logplugin","log",""))
    self.log.add("kf")
    self.log.add("st")
    self.log.sendMsg(":max-length 80")
    self.log.start()

    # Specific to walking
    hstsetup.stsetup(self.st)
    self.kf.start()
    self.st.start()


  #
  # Initial position
  #
  def GoInitialPosition(self):

    # Half sitting 
    rleg_a  = "0 0 -26 50 -24 0"
    lleg_a  = "0 0 -26 50 -24 0"
    chest_a = "0 0"
    head_a  = "0 0"
    rarm_a  = "15 -10 0 -30 0 0 10"
    larm_a  = "15  10 0 -30 0 0 10"
    rhand_a = "-10.0 10.0 -10.0 10.0 -10.0"
    lhand_a = "-10.0 10.0 -10.0 10.0 -10.0"

    # Custom
    chest_a = "0 -3"

    # New Germany
    # rarm_a = "33 -10 0 -120 -10 0 10" # <-- ballrose: "10 -10 -10 -80 0 -0 0", dunnowhat: "10 -10 -10 -70 0 -0 10"
    # larm_a = "33  10 0 -120  10 0 10" # <-- ballrose: "35 5 0 -20.0 0 0 10.0", dunnowhat: "10  10 10 -70 0 0 10"
   
    # Old Germany
    # rarm_a = "10 -18 0 -100 -18 0 10"
    # larm_a = "10  18 0 -100  18 0 10"

    self.seq.sendMsg(":joint-angles "+rleg_a+" "+lleg_a+" "+chest_a+" "+head_a+" "+rarm_a+" "+larm_a+" "+rhand_a+" "+lhand_a+" 2.5")
    self.seq.waitInterpolation()


  #
  # Initialize the Stack of Tasks
  #
  def InitializeSoT(self):

    self.SoT.sendMsg(":init")

    # teleop runs with dynsmall, NOT with standard dyn
    self.SoT.sendMsg(":script run /home/blue/sot-lib/script/dynsmall")
    self.SoT.sendMsg(":script run /home/blue/sot-lib/script/coshell")
    self.SoT.sendMsg(":script run /home/blue/sot-lib/script/traces")

    self.SoT.sendMsg(":script plug OpenHRP.state dyn.position")
    self.SoT.sendMsg(":script plug OpenHRP.state dyn2.position")
    self.SoT.sendMsg(":script plug OpenHRP.attitude posKF.attitudeIN")
    self.SoT.sendMsg(":script plug OpenHRP.attitude flex.sensorWorldRotation")
    self.SoT.sendMsg(":script plug sot.control OpenHRP.control")

    self.SoT.sendMsg(":script run /home/blue/sot-lib/script/force")
    self.SoT.sendMsg(":script run /home/blue/sot-lib/script/forceL")
    self.SoT.sendMsg(":script run /home/blue/sot-lib/script/small")
    self.SoT.sendMsg(":script run /home/blue/sot-lib/script/jointlimit")
    # self.SoT.sendMsg(":script run /home/blue/sot-lib/script/collisiondetection")
    self.SoT.sendMsg(":script run /home/blue/sot-lib/script/grip")

    self.SoT.sendMsg(":script OpenHRP.pause")
    self.SoT.start()

    self.SoT.sendMsg(":script run /home/blue/sot-lib/script/hwpg")
    self.SoT.sendMsg(":script run /home/blue/sot-lib/script/dynmc")
    self.SoT.sendMsg(":script run /home/blue/sot-lib/script/teleoperation/teleop")
    self.SoT.sendMsg(":script sot.clear")


  #
  # Starts writing the control output of the SoT
  #
  def Play(self):

    self.SoT.sendMsg(":script OpenHRP.reinit from mc")
    self.SoT.sendMsg(":script OpenHRP.play")
    
    if(self.with_trace):
      self.SoT.sendMsg(":script tr.start")

    waitInputConfirm("click [OK] to continue.")


  #
  # Initializes higher-level components related to the PG
  #
  def LaunchWalking(self):

    self.SoT.sendMsg(":script run /home/blue/sot-lib/script/walking/hwpgjointsbscw")
    self.SoT.sendMsg(":script run /home/blue/sot-lib/script/walkreact")

    waitInputConfirm("Click [OK] to continue.")


  #
  # Cleans everything and log data
  #
  def StopAndCleanup(self):

    # Careful! The SOT plugin should not be stopped before seqplay has ended its
    # motion. waitInterpolation should block the script. If not, don't push [OK] to
    # the next dialog box before the robot has reached the 1/2seating
    self.SoT.sendMsg(":hold")

    if(self.with_trace):
      self.SoT.sendMsg(":script tr.trace")

    self.SoT.sendMsg(":waitForFinished")
    self.SoT.stop()
    self.SoT.sendMsg(':profile')

    self.ms.sendMsg(":destroy SoT")
    self.ms.sendMsg(":unload StackOfTasks")

    self.log.stop()
    self.log.save("sot-plugin")

    print("Script finished")


Dialog = StackOfTasksDialog("Stack Of Tasks")

defaultMenuE = [[
      '------- Sequence ----------', '#label',
      '1. Play',                     'Dialog.Play()',
      '2. LaunchWalking',            'Dialog.LaunchWalking()',
      '3. StopAndCleanup',           'Dialog.StopAndCleanup()'
    ]]

waitInputMenu(defaultMenuE)
