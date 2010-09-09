loadPlugin pg${DYN_LIB_EXT}
loadPlugin exception-pg${DYN_LIB_EXT}
new PatternGenerator pg

# Initialize files and information of the pattern generator.
pg.setParamPreview ${HRP2_CONFIG_DIRECTORY}/PreviewControlParameters.ini
pg.setVrmlDir ${HRP2_MODEL_DIRECTORY}/
pg.setVrml HRP2JRLmainsmall.wrl 
pg.setXmlSpec ${HRP2_CONFIG_DIRECTORY}/HRP2SpecificitiesSmall.xml
pg.setXmlRank ${HRP2_CONFIG_DIRECTORY}/HRP2LinkJointRankSmall.xml 

# Build internal object
pg.buildModel

# Standard initialization
pg.parsecmd :samplingperiod 0.005
pg.parsecmd :previewcontroltime 1.6
pg.parsecmd :comheight 0.814
pg.parsecmd :omega 0.0
pg.parsecmd :stepheight 0.05
pg.parsecmd :singlesupporttime 0.780
pg.parsecmd :doublesupporttime 0.020
pg.parsecmd :armparameters 0.5
pg.parsecmd :LimitsFeasibility 0.0
pg.parsecmd :ZMPShiftParameters 0.015 0.015 0.015 0.015 
pg.parsecmd :TimeDistributeParameters 2.0 3.5 1.0 3.0
pg.parsecmd :UpperBodyMotionParameters 0.0 -0.5 0.0 
pg.parsecmd :comheight 0.814
pg.parsecmd :SetAlgoForZmpTrajectory Morisawa

# Plug OpenHRP in the pg to get the current state of the robot.
plug OpenHRP.state pg.position
# plug OpenHRP.motorcontrol pg.motorcontrol
plug OpenHRP.previousState pg.motorcontrol
plug OpenHRP.zmppreviouscontroller pg.zmppreviouscontroller
plug dyn.com pg.com




