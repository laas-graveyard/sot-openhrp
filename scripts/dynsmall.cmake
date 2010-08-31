run ${CMAKE_INSTALL_PREFIX}/script/base
echo Creating the small dynamic
new DynamicHrp2 dyn
new DynamicHrp2 dyn2
run ${CMAKE_INSTALL_PREFIX}/script/dynfilessmall
dyn2.parse
new VectorConstant zero
zero.resize 36
plug zero.out dyn2.position
plug zero.out dyn2.velocity
plug zero.out dyn2.acceleration

new VectorConstant dyn2ffposzero
dyn2ffposzero.resize 6
dyn2ffposzero.[] 0 0.0
dyn2ffposzero.[] 1 0.0
dyn2ffposzero.[] 2 0.0
dyn2ffposzero.[] 3 0.0
dyn2ffposzero.[] 4 0.0
dyn2ffposzero.[] 5 0.0

plug dyn2ffposzero.out dyn2.ffposition

dyn2.createOpPoint 0 22
dyn2.createOpPoint lh 29
dyn2.createOpPoint rleg 6 
dyn2.createOpPoint lleg 12 
dyn2.createOpPoint chest 14

dyn2.setProperty ComputeVelocity               false
dyn2.setProperty ComputeCoM                    false
dyn2.setProperty ComputeAccelerationCoM        false
dyn2.setProperty ComputeMomentum               false
dyn2.setProperty ComputeZMP                    false
dyn2.setProperty ComputeBackwardDynamics       false

set dyn.gearRatio [36](0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,207.69,381.54,0,0,219.23,231.25,266.67,250.0,145.45,350.0,0,0,0,0,0,0,0,0)
set dyn.inertiaRotor [36](0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,69.6e-7,69.6e-7,0,0,69.6e-7,66.0e-7,10.0e-7,66.0e-7,11.0e-7,10.0e-7,0,0,0,0,0,0,0,0)

# ----------------------------------------------------
# --- FLEX -------------------------------------------
# ----------------------------------------------------

# ------- Flex based kinematics
new AngleEstimator flex
flex.fromSensor false
plug dyn2.lleg flex.contactEmbeddedPosition
plug dyn2.chest flex.sensorEmbeddedPosition

new RPYToMatrix attitudeSensor
# plug attitudeSensor.out flex.sensorWorldRotation
plug flex.waistWorldPoseRPY dyn.ffposition

# --- Flexibility velocity
new Derivator<Vector> flexV
plug flex.angles flexV.in

# --- PosFF from leg contact + sensor # DEPRECIATED
new WaistPoseFromSensorAndContact posKF
plug dyn2.lleg posKF.contact
plug dyn2.chest posKF.position
posKF.fromSensor true

# --- DYN With true posFF	
dyn.parse
plug zero.out dyn.velocity
plug zero.out dyn.acceleration
# plug posKF.positionWaist dyn.ffposition
plug flex.waistWorldPoseRPY dyn.ffposition

# ----------------------------------------------------
# --- TASKS ------------------------------------------
# ----------------------------------------------------

# -- TASK Manip
dyn.createOpPoint 0 22
# dyn.createOpPoint 0 34
# dyn.createOpPoint 0 0
dyn.createOpPoint rleg 6 
dyn.createOpPoint lleg 12 

new FeaturePoint6d p6
new FeaturePoint6d p6d

new Compose<R+T> comp
new MatrixConstant eye3
eye3.resize 3 3
eye3.eye
plug eye3.out comp.in1
new VectorConstant t
t.resize 3
# WAIST
t.[] 0 0
t.[] 1 0.095
t.[] 2 0.563816
# HAND
t.[] 0 0.25
t.[] 1 -0.5
t.[] 2 .85
plug t.out comp.in2

plug comp.out p6d.position
plug dyn.J0 p6.Jq
plug dyn.0 p6.position
set p6.sdes p6d

new Task task
task.add p6
new GainAdaptative gain
gain.setConstant .2
plug task.error gain.error
plug gain.gain task.controlGain

eye3.[] 0 0 0
eye3.[] 2 2 0
eye3.[] 0 2 -1
eye3.[] 2 0 1

set p6.selec 000111
p6.frame current

# --- COM 
dyn.setComputeCom 1
# dyn.setProperty ComputeVelocity true
# dyn.setProperty ComputeMomentum true
# dyn.setProperty ComputeZMP true

new FeatureGeneric featureCom
plug dyn.com featureCom.errorIN
plug dyn.Jcom featureCom.jacobianIN 
# set featureCom.selec 111
	
new FeatureGeneric featureComDes
# set featureComDes.errorIN [2](0,-0)
set featureCom.sdes featureComDes

new Task taskCom
taskCom.add featureCom
set taskCom.controlGain .3
# set taskCom.controlGain 1

# --- CHEST
new FeatureGeneric featureChest
set featureChest.jacobianIN [2,36]((0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0),(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0))
set featureChest.errorIN [2](0,0)
new Task taskChest
taskChest.add featureChest
set taskChest.controlGain 1

# --- Task Chest for blocking the chest joints
# new FeatureGeneric featureChest
# set featureChest.jacobianIN [2,36]((0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0),(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0))
# set featureChest.errorIN [2](0,0)
# new Task taskChest
# taskChest.add featureChest
# set taskChest.controlGain 1

# --- Task RightArm for blocking the right arm
new FeatureGeneric featureRightArm
set featureRightArm.jacobianIN [7,36]((0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0),(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0),(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0),(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0),(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0),(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0),(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0))
set featureRightArm.errorIN [7](0,0,0,0,0,0,0)
new Task taskRightArm
taskRightArm.add featureRightArm
set taskRightArm.controlGain 1

# --- Task LeftArm for  blocking the left arm
new FeatureGeneric featureLeftArm
set featureLeftArm.jacobianIN [7,36]((0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0),(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0),(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0),(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0),(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0),(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0),(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1))
set featureLeftArm.errorIN [7](0,0,0,0,0,0,0)
new Task taskLeftArm
taskLeftArm.add featureLeftArm
set taskLeftArm.controlGain 1

# --- JL
new FeatureJointLimits featureJl
featureJl.actuate
plug dyn.position featureJl.joint
plug dyn.upperJl featureJl.upperJl
plug dyn.lowerJl featureJl.lowerJl
new Task taskJl
taskJl.add featureJl
set taskJl.controlGain .1

# --- LEGS
new FeatureGeneric featureLegs
new MatrixConstant jacobianLegs
jacobianLegs.resize 12 36
jacobianLegs.[] 0 6 1
jacobianLegs.[] 1 7 1
jacobianLegs.[] 2 8 1
jacobianLegs.[] 3 9 1
jacobianLegs.[] 4 10 1
jacobianLegs.[] 5 11 1
jacobianLegs.[] 6 12 1
jacobianLegs.[] 7 13 1
jacobianLegs.[] 8 14 1
jacobianLegs.[] 9 15 1
jacobianLegs.[] 10 16 1
jacobianLegs.[] 11 17 1
plug jacobianLegs.out featureLegs.jacobianIN 

new VectorConstant vectorLegs
vectorLegs.resize 12
vectorLegs.fill 0
plug vectorLegs.out featureLegs.errorIN
# set featureLegs.errorIN [12](0,0,0,0,0,0,0,0,0,0,0,0)
new Task taskLegs
taskLegs.add featureLegs
set taskLegs.controlGain 1

# --- TWOFEET
new FeaturePoint6dRelative featureTwofeet
plug dyn.Jrleg  featureTwofeet.Jq 
plug dyn.Jlleg  featureTwofeet.JqRef
plug dyn.rleg  featureTwofeet.position 
plug dyn.lleg  featureTwofeet.positionRef
# set featureTwofeet.error [6](0,0,0,0,0,0)

new Task taskTwofeet
taskTwofeet.add featureTwofeet
set taskTwofeet.controlGain 0

# --- CONTACT CONSTRAINT
new Constraint legs
legs.add dyn.Jlleg

# --- SOT
new SOT sot
set sot.damping 1e-6
sot.addConstraint legs
sot.nbJoints 36

# plug dyn.inertia sot.weight
# sot.push taskTwofeet
# sot.push taskLegs
# sot.push task
# sot.push taskCom
# sot.push taskChest
# sot.gradient taskJl

set featureComDes.errorIN [3](0.004,-0.09,.6)
dispmat matlab

new VectorConstant zeroCom
zeroCom.resize 36
zeroCom.fill 0

dyn.createOpPoint Waist 0 