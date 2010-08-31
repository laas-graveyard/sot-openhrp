loadPlugin sotStepChecker${DYN_LIB_EXT}
loadPlugin sotStepObserver${DYN_LIB_EXT}
loadPlugin sotStepQueue${DYN_LIB_EXT}
loadPlugin sotStepComputerForce${DYN_LIB_EXT}
loadPlugin sotStepComputerJoystick${DYN_LIB_EXT}
loadPlugin sotPGManager${DYN_LIB_EXT}
loadPlugin sotStepTimeLine${DYN_LIB_EXT}

new StepQueue stepqueue
new StepComputerJoystick stepcomp
new PGManager steppg
new TimeLine stepper

stepper.setComputer stepcomp
stepper.setPGManager steppg
stepper.setQueue stepqueue

plug pg.SupportFoot stepcomp.contactfoot

# Emergency box: in case of bug, break the commentary and check.
# stepper.position
# signalTime stepper.position

steppg.initPg pg

OpenHRP.periodicCall addSignal stepper.trigger
OpenHRP.periodicCall addSignal stepcomp.laststep
