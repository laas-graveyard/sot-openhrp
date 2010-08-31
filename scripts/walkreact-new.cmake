loadPlugin sotStepChecker${DYN_LIB_EXT}
loadPlugin sotStepObserver${DYN_LIB_EXT}
loadPlugin sotStepQueue${DYN_LIB_EXT}
loadPlugin sotStepComputerPos${DYN_LIB_EXT}
loadPlugin sotStepComputerJoystick${DYN_LIB_EXT}
loadPlugin sotPGManager${DYN_LIB_EXT}
loadPlugin sotStepTimeLine${DYN_LIB_EXT}

new StepObserver stepobs
new StepQueue stepqueue
new StepComputerPos stepcomp
new PGManager steppg
new TimeLine stepper

stepcomp.setObserver stepobs
stepper.setComputer stepcomp
stepper.setPGManager steppg
stepper.setQueue stepqueue

plug dyn.lh stepobs.lefthand
plug dyn.0 stepobs.righthand
plug dyn.lleg stepobs.leftfoot
plug dyn.rleg stepobs.rightfoot
plug dyn.Waist stepobs.waist

plug pg.SupportFoot stepcomp.contactfoot
plug stepobs.position2handLeft stepcomp.posrefleft
plug stepobs.position2handRight stepcomp.posrefright

OpenHRP.periodicCall addSignal stepobs.position2handLeft
OpenHRP.periodicCall addSignal stepobs.position2handRight
compute stepcomp.posrefleft
compute stepcomp.posrefright

stepcomp.thisIsZero record
steppg.initPg pg
OpenHRP.periodicCall addSignal stepper.trigger
