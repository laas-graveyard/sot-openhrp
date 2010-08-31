run ${CMAKE_INSTALL_PREFIX}/script/simusmall
run ${CMAKE_INSTALL_PREFIX}/script/traces

plug OpenHRP.state dyn.position
plug OpenHRP.state dyn2.position
plug OpenHRP.attitude posKF.attitudeIN
plug OpenHRP.attitude flex.sensorWorldRotation
plug sot.control OpenHRP.control

run ${CMAKE_INSTALL_PREFIX}/script/force
run ${CMAKE_INSTALL_PREFIX}/script/forceL
run ${CMAKE_INSTALL_PREFIX}/script/small
run ${CMAKE_INSTALL_PREFIX}/script/jointlimit
run ${CMAKE_INSTALL_PREFIX}/script/grip

run ${CMAKE_INSTALL_PREFIX}/script/teleoperation/teleop
sot.clear

run ${CMAKE_INSTALL_PREFIX}/script/hwpgpginit
run ${CMAKE_INSTALL_PREFIX}/script/hwpginitframes
run ${CMAKE_INSTALL_PREFIX}/script/walkreact
run ${CMAKE_INSTALL_PREFIX}/script/walking/hwpgfeettasksrel


sot.clear
sot.push taskForce
sot.push taskForceLH



tr.clear
tr.add Aact.out inertia_sot



set dyn.position [36](0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0)
set dyn2.position [36](0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0)


chrono OpenHRP.inc

tr.stop
tr.trace

