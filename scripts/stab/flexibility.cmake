
new Inverse<matrixhomo> contactMwaist
plug posKF.contact contactMwaist.in
new MatrixHomoToPoseRollPitchYaw contactTRwaist
plug contactMwaist.out contactTRwaist.in

tr.add contactTRwaist.out attitudeIK

new PoseRollPitchYawToMatrixHomo poseWaist
plug posKF.positionWaist poseWaist.in
new Multiply<matrixhomo> diffAttitude
plug poseWaist.out diffAttitude.in1
plug posKF.contact diffAttitude.in2
new MatrixHomoToPoseRollPitchYaw diffAttitudeVect
plug diffAttitude.out diffAttitudeVect.in
tr.add diffAttitudeVect.out diffAttitude

sot.push taskTwofeet
sot.push taskCom

set featureComDes.errorIN [3](0.004,-0.09,.7)
set featureComDes.errorIN [3](0.003957,-0.0933716,0.702736)
set featureComDes.errorIN [3](0.0,0.0,0.702736)
set featureComDes.errorIN [3](0.01,0.0153,0.702736)


# set featureComDes.errorIN [3](0.000,-0.00,.6)

OpenHRP.periodicCall addSignal contactTRwaist.out 
OpenHRP.periodicCall addSignal diffAttitudeVect.out 


# Leve le pied
plug dyn.Jrleg p6.Jq
plug dyn.rleg p6.position
t.[] 0 0
t.[] 1 -0.199453
t.[] 2 0.1
set task.controlGain .05
set taskCom.controlGain 1

new MatrixHomoToPoseUTheta poseUT
plug poseWaist.out poseUT.in

sot.push task

# --- STAB ---
dyn.createPosition waist 0
new ZmprefFromCom zmp
plug dyn.waist zmp.waist
plug dyn.com zmp.com
plug taskCom.task zmp.dcom

plug zmp.zmpref OpenHRP.zmp

new VectorConstant dcom0
dcom0.resize 3
dcom0.fill 0.
plug dcom0.out zmp.dcom
plug featureComDes.errorIN zmp.com

tr.add zmp.zmpref



# ---
# posKF.fromSensor false

new Dynamic dynSL
dynSL.setFiles  ${OPENHRP_HOME}/etc/HRP2JRL/ HRP2JRLmain.wrl ${OPENHRP_HOME}/Controller/IOserver/robot/HRP2JRL/bin/HRP2Specificities.xml ${OPENHRP_HOME}/Controller/IOserver/robot/HRP2JRL/bin/HRP2LinkJointRank.xml 
dynSL.parse
new WaistPoseFromSensorAndContact posKFSL
plug dyn2.lleg posKFSL.contact
plug dyn2.chest posKFSL.position
posKFSL.fromSensor false
plug zero.out dynSL.velocity
plug zero.out dynSL.acceleration
plug posKFSL.positionWaist dynSL.ffposition
dynSL.setComputeCom 1

plug OpenHRP.state dynSL.position
plug OpenHRP.attitude posKFSL.attitudeIN
plug dynSL.com featureCom.errorIN

dynSL.createPosition waist 0
plug dynSL.waist zmp.waist

plug dynSL.Jcom featureCom.jacobianIN 

proc step
# -> unplug zmp.waist
-> sot.rm taskTwofeet
-> set task.controlGain 1.
-> t.[] 0 .3
-> t.[] 2 .15
-> proc step 
-> -> t.[] 2 .007
-> -> set task.controlGain .6
-> endproc
endproc
