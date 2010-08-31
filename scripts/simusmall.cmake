run ${CMAKE_INSTALL_PREFIX}/script/dynsmall

new RobotSimu OpenHRP
OpenHRP.resize 36
OpenHRP.set [36](0,0,0,0,0,0,0,0,-1.04720,2.09440,-1.04720,0,0,0,-1.04720,2.09440,-1.04720,0,0.0000,0,-0,-0,0.17453,-0.17453,-0.17453,-0.87266,0,-0,0.1,0.17453,-0.17453,-0.17453,-0.87266,0,-0,0.1)

plug sot.control OpenHRP.control
plug OpenHRP.state dyn.position
plug OpenHRP.state dyn2.position
plug OpenHRP.attitude posKF.attitudeIN
plug OpenHRP.attitude flex.sensorWorldRotation
# plug OpenHRP.attitude attitudeSensor.in

