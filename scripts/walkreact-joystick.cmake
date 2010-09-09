loadPlugin step-checker${DYN_LIB_EXT}
loadPlugin step-observer${DYN_LIB_EXT}
loadPlugin step-queue${DYN_LIB_EXT}
loadPlugin step-computer-force${DYN_LIB_EXT}
loadPlugin step-computer-joystick${DYN_LIB_EXT}
loadPlugin pg-manager${DYN_LIB_EXT}
loadPlugin step-time-line${DYN_LIB_EXT}

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
