run ${CMAKE_INSTALL_PREFIX}/script/dyn

new RobotSimu OpenHRP
OpenHRP.resize 46
OpenHRP.set [46](0,0,0,0,0,0,0,0,-1.04720,2.09440,-1.04720,0,0,0,-1.04720,2.09440,-1.04720,0,0.0000,0,-0,-0,0.17453,-0.17453,-0.17453,-0.87266,0,-0,0.1,0.17453,-0.17453,-0.17453,-0.87266,0,-0,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1)
# Position zero
# OpenHRP.set [46](0,0,0,0,0,0,0,0,-0,0,-0,0,0,0,-0,0,-0,0.0000,0,0,-0,0.0000,0,0,-0,0,-0,0,-0.000000,0,0,-0,0,-0,0,0.00000,0,0,0,0,0,0,0,0,0,0)
# Position symetric right/left arms
OpenHRP.set [46](0,0,0,0,0,0,0,0,-1.04720,2.09440,-1.04720,0,0,0,-1.04720,2.09440,-1.04720,0,0.0000,0,-0,-0,0.17453,-0.17453,-0.17453,-0.87266,0,-0,.0999,0.17453,0.17453,0.17453,-0.87266,0,-0,.0999,.0999,.0999,.0999,.0999,.0999,.0999,.0999,.0999,.0999,.0999)

plug sot.control OpenHRP.control
plug OpenHRP.state dyn.position
plug OpenHRP.state dyn2.position
plug OpenHRP.attitude posKF.attitudeIN
plug OpenHRP.attitude flex.sensorWorldRotation
# plug OpenHRP.attitude attitudeSensor.in

