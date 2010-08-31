
# --- To compute ZMP -------------
# new DynamicHrp2 dyn3
# dyn3.setFiles ${HRP2_MODEL_DIRECTORY}/ HRP2JRLmainsmall.wrl ${HRP2_CONFIG_DIRECTORY}/HRP2Specificities.xml ${HRP2_CONFIG_DIRECTORY}/HRP2LinkJointRankSmall.xml
# dyn3.parse

new Derivator<Vector> qdot
plug dyn.position qdot.in
set qdot.dt 200

new Derivator<Vector> qddot
plug qdot.out qddot.in
set qddot.dt 200

new Derivator<Vector> ffdot
plug dyn.ffposition ffdot.in
set ffdot.dt 200

# plug dyn.position dyn3.position
# plug qdot.out dyn3.velocity
# plug qddot.out dyn3.acceleration
# plug dyn.ffposition dyn3.ffposition
# plug ffdot.out dyn3.ffvelocity

# --------------------------------

# ---- WAIST TASK ----
new FeaturePoint6d WaistFeature
new FeaturePoint6d WaistFeatureDesired

plug dyn.Waist WaistFeature.position
plug dyn.JWaist WaistFeature.Jq

set WaistFeature.sdes WaistFeatureDesired

new VectorConstant vectorCom
vectorCom.resize 3
vectorCom.[] 0 0.0
vectorCom.[] 1 0.0
vectorCom.[] 2 0.0

new Multiply<vector> comfiltered
plug vectorCom.out comfiltered.in1
plug pg.comref comfiltered.in2

new Add<vector> WaistPCom
plug pg.initwaistposref WaistPCom.in1
plug comfiltered.out WaistPCom.in2

new Stack<vector> WaistPose
plug WaistPCom.out WaistPose.in1
plug pg.initwaistattref WaistPose.in2
WaistPose.selec1 0 3
WaistPose.selec2 0 3

new PoseRollPitchYawToMatrixHomo MHWaistPose
plug WaistPose.out MHWaistPose.in
plug MHWaistPose.out WaistFeatureDesired.position

new Task taskWaist
taskWaist.add WaistFeature

set WaistFeature.selec 011100
set taskWaist.controlGain 5

# --- ANGULAR MOMENTUM TASK ---
# new FeatureGeneric featureAngularMomentum
# featureAngularMomentum.dimDefault 3
# plug dyn3.angularmomentum featureAngularMomentum.errorIN
# plug dyn3.Jangularmomentum featureAngularMomentum.jacobianIN
# set featureAngularMomentum.selec 011

# new Task taskAngularMomentum
# taskAngularMomentum.add featureAngularMomentum
# set taskAngularMomentum.controlGain -20

# --- COM REF ---

new Multiply<vector,matrixHomo> pg_comref
plug pg.comref pg_comref.in1
plug lfo_H_pg.out pg_comref.in2

# --- Selector of Com Ref and Foot Ref --- 
new Selector PGselec
PGselec.reset 2 1
PGselec.create vector scomref 0

# Dyn by default
plug dyn.com PGselec.scomref0
plug pg_comref.out PGselec.scomref1

# Plug the left leg.

new Multiply<matrixhomo> pg_leftfootref
plug pg.leftfootref pg_leftfootref.in1
plug lfo_H_pg.out pg_leftfootref.in2

# Plug the right leg.
new Multiply<matrixhomo> pg_rightfootref
plug pg.rightfootref pg_rightfootref.in1
plug lfo_H_pg.out pg_rightfootref.in2

# when the PG is processing.
plug pg.inprocess PGselec.selec

# Send the reference command to the task.
plug PGselec.scomref featureComDes.errorIN

set featureComDes.selec 011
set featureCom.selec 011

set taskCom.controlGain 1

# --- Handling the two feet ---

# switch between the two ref 
# depending on who is in contact with the floor.

new Selector RefFeetRelselec
RefFeetRelselec.reset 2 6

RefFeetRelselec.create matrixHomo DesFoot 0
plug pg_rightfootref.out RefFeetRelselec.DesFoot0
plug pg_leftfootref.out RefFeetRelselec.DesFoot1

RefFeetRelselec.create matrixHomo DesRefFoot 1
plug pg_leftfootref.out RefFeetRelselec.DesRefFoot0
plug pg_rightfootref.out RefFeetRelselec.DesRefFoot1

RefFeetRelselec.create matrixHomo Foot 2
plug dyn.rleg RefFeetRelselec.Foot0
plug dyn.lleg RefFeetRelselec.Foot1

RefFeetRelselec.create matrixHomo RefFoot 3
plug dyn.lleg RefFeetRelselec.RefFoot0
plug dyn.rleg RefFeetRelselec.RefFoot1

RefFeetRelselec.create matrix JFoot 4
plug dyn.Jrleg RefFeetRelselec.JFoot0
plug dyn.Jlleg RefFeetRelselec.JFoot1

RefFeetRelselec.create matrix JRefFoot 5
plug dyn.Jlleg RefFeetRelselec.JRefFoot0
plug dyn.Jrleg RefFeetRelselec.JRefFoot1

plug pg.SupportFoot RefFeetRelselec.selec
# set RefFeetRelselec.selec 0

new FeaturePoint6dRelative featureTwofeetDes
featureTwofeet.initSdes featureTwofeetDes
set featureTwofeet.sdes featureTwofeetDes

# unplug featureTwofeet.position
# unplug featureTwofeet.positionRef
# unplug featureTwofeetDes.position
# unplug featureTwofeetDes.positionRef

plug RefFeetRelselec.Foot featureTwofeet.position
plug RefFeetRelselec.RefFoot featureTwofeet.positionRef
plug RefFeetRelselec.JFoot featureTwofeet.Jq
plug RefFeetRelselec.JRefFoot featureTwofeet.JqRef
plug RefFeetRelselec.DesFoot featureTwofeetDes.position
plug RefFeetRelselec.DesRefFoot featureTwofeetDes.positionRef

# --- Creating PD tasks ---
new TaskPD taskTwofeetPD
taskTwofeetPD.add featureTwofeet

plug pg.dotrightfootref featureTwofeetDes.dotposition
plug pg.dotleftfootref featureTwofeetDes.dotpositionRef
plug featureTwofeetDes.errordot taskTwofeetPD.errorDot

new TaskPD taskComPD
taskComPD.add featureCom
plug pg.dcomref featureComDes.errordotIN
plug featureCom.errordot taskComPD.errorDot

# --- Creating task for the waist

# --- Adaptative Gain ---
# new GainAdaptative gainAdapCom
# gainAdapCom.set 5 .5 50
# plug gainAdapCom.gain taskCom.controlGain
# plug taskCom.error gainAdapCom.error

# new GainAdaptative gainAdapTwofeet
# gainAdapTwofeet.set 5 .5 100
# plug gainAdapTwofeet.gain taskTwofeet.controlGain
# plug taskTwofeet.error gainAdapTwofeet.error
set taskCom.controlGain 40

set taskComPD.controlGain 40
taskComPD.beta -1

# Good in kinematics - Bad in dynamics
# set taskTwofeet.controlGain 200
set taskTwofeet.controlGain 200

# set taskTwofeetPD.controlGain 1

# --- Error dot computation --

# --- SOT ---
set sot.damping 0.05
sot.clear

# Handling constraints
sot.clearConstraint

legs.clear
legs.add RefFeetRelselec.JFoot
sot.addConstraint legs

# -- SOT: PUSHING THE TASKS 

# -- CoM
sot.push taskComPD

# -- Legs 
sot.push taskTwofeet

# -- Waist
sot.push taskWaist

# --- TRACE ---

OpenHRP.periodicCall addSignal pg_comref.out
OpenHRP.periodicCall addSignal dyn.com
OpenHRP.periodicCall addSignal taskCom.error
OpenHRP.periodicCall addSignal pg.dcomref
# OpenHRP.periodicCall addSignal dyn2.momenta

tr.add taskCom.error errorcom
tr.add taskCom.task taskcom
tr.add dyn.com com
tr.add pg_comref.out comref
tr.add taskCom.controlGain gaincom
tr.add pg.comref comref0
tr.add pg.zmpref zmpref
tr.add OpenHRP.zmp ohzmpref
tr.add lfo_H_wa.out lfoHwa
tr.add lfo_H_zmp.out lfoHzmp
tr.add lfo_H_pg.out lfoHpg
tr.add pg_H_wa.out pgHwa
tr.add WaistPose.out waistpose

tr.add SupportFootSelec.pg_H_sf pg_H_sf
tr.add sf_H_wa.out sf_H_wa

tr.add pg.dcomref dcomref
tr.add featureCom.errordot comerrordot

tr.add taskTwofeet.error errortwofeet
tr.add taskTwofeetPD.error errortwofeetPD
# tr.add taskWaist.error errorwaist

tr.add RefFeetRelselec.Foot pgsfoot
tr.add RefFeetRelselec.RefFoot pgsreffoot
tr.add RefFeetRelselec.DesFoot pgsdesfoot
tr.add RefFeetRelselec.DesRefFoot pgsdesreffoot

OpenHRP.periodicCall addSignal pg.initleftfootref
OpenHRP.periodicCall addSignal pg.initrightfootref
tr.add pg.leftfootref pgleftfootref
tr.add pg.rightfootref pgrightfootref
tr.add pg.initleftfootref pginitleftfootref
tr.add pg.initrightfootref pginitrightfootref

tr.add dyn.lleg dynleftfoot
tr.add dyn.rleg dynrightfoot
tr.add dyn2.lleg dyn2leftfoot
tr.add dyn2.rleg dyn2rightfoot

# OpenHRP.periodicCall addSignal dyn3.angularmomentum
# tr.add dyn3.angularmomentum angularmomentum

# dyn3.setProperty ComputeZMP true
# tr.add dyn.momenta dynmomenta

# OpenHRP.periodicCall addSignal dyn2.ffposition

# tr.add dyn3.zmp dyn3zmp
# OpenHRP.periodicCall addSignal dyn.momenta

tr.add dyn.ffposition dynffposition
tr.add dyn2.ffposition dyn2ffposition
tr.add pg.SupportFoot supportfoot
tr.add ffattitude_from_pg.out ffattitude_from_pg
# tr.add featureHeadWPG.position headwpgpos
# tr.add featureHeadWPGDes.position headwpgposdes

tr.start

# stepper.state start

# --- DEBUG ---
# plug controlsmall.out OpenHRP.control
# set taskCom.controlGain 1

# plug flex.waistWorldPoseRPY dyn.ffposition

# set taskCom.controlGain 15
# set taskCom.controlGain 100

# new GainHyperbolic gainComH
# gainComH.set 15 1 8e3 .02
# plug taskCom.error gainComH.error
# plug gainComH.gain taskCom.controlGain

# set taskCom.controlGain 15

# --- 
# tr.add dyn.ffposition 
# stepper.state start


new CoMFreezer freezercom
plug pg.inprocess freezercom.PGInProcess
plug dyn.com freezercom.CoMRef
plug freezercom.freezedCoM PGselec.scomref0

OpenHRP.periodicCall addSignal freezercom.freezedCoM

