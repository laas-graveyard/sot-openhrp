loadPlugin HumanoidRobotSCD${DYN_LIB_EXT}
loadPlugin collision-detector${DYN_LIB_EXT}

new CollisionDetector collision

collision.addPair rleg0 lleg0

collision.addPair rleg2 lleg2
collision.addPair rleg3 lleg3
collision.addPair rleg5 lleg5
collision.addPair rleg3 lleg5
collision.addPair rleg5 lleg3


collision.jointsToFreeze 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29

collision.threshold 0.003

plug dyn.position collision.joint
squeeze OpenHRP.control collision.controlIN collision.control
