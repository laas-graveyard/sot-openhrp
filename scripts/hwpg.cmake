loadPlugin pg${DYN_LIB_EXT}
loadPlugin exception-pg${DYN_LIB_EXT}
new PatternGenerator pg

# Initialize files and information of the pattern generator.
pg.setParamPreview ${HRP2_CONFIG_DIRECTORY}/PreviewControlParameters.ini
pg.setVrmlDir ${HRP2_MODEL_DIRECTORY}/
pg.setVrml HRP2JRLmain.wrl 
pg.setXmlSpec ${HRP2_CONFIG_DIRECTORY}/HRP2Specificities.xml
pg.setXmlRank ${HRP2_CONFIG_DIRECTORY}/HRP2LinkJointRank.xml 

# Build internal object
pg.buildModel

# Standard initialization
pg.parsecmd :omega 0.0
pg.parsecmd :stepheight 0.05
pg.parsecmd :singlesupporttime 0.78
pg.parsecmd :doublesupporttime 0.02
pg.parsecmd :armparameters 0.5
pg.parsecmd :LimitsFeasibility 0.0
pg.parsecmd :ZMPShiftParameters 0.015 0.015 0.015 0.015 
pg.parsecmd :TimeDistributeParameters 2.0 3.5 1.0 3.0
pg.parsecmd :UpperBodyMotionParameters 0.0 -0.5 0.0 
pg.parsecmd :SetAlgoForZmpTrajectory Kajita

# Plug OpenHRP in the pg to get the current state of the robot.
plug OpenHRP.state pg.position
plug OpenHRP.motorcontrol pg.motorcontrol
plug OpenHRP.zmppreviouscontroller pg.zmppreviouscontroller
plug dyn.com pg.com
plug pg.zmpref OpenHRP.zmp
plug pg.waistattitude OpenHRP.attitudeIN

# ------------------------------------------------------------
# Modify the origin to ensure continuity with and w/ the walk.
# ------------------------------------------------------------
new Inverse<matrixhomo> lfo_H_pg
plug dyn2.lleg lfo_H_pg.in
compute lfo_H_pg.out
freeze lfo_H_pg.in

new Compose<RPY+T> pg_H_wa
plug pg.waistpositionabsolute pg_H_wa.in2
plug pg.waistattitudeabsolute pg_H_wa.in1

new Multiply<matrixhomo> lfo_H_wa
plug lfo_H_pg.out lfo_H_wa.in1
plug pg_H_wa.out lfo_H_wa.in2

new MatrixHomoToPoseRollPitchYaw ffpos_from_pg
plug lfo_H_wa.out ffpos_from_pg.in

# This last line will pass the dyn from ref left_foot to ref pg.
plug ffpos_from_pg.out dyn.ffposition

# --- Last init and you can go.
pg.initState

