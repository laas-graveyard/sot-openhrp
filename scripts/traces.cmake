new TracerRealTime tr
tr.bufferSize 10485760

tr.open  ${TRACE_REPOSITORY} jl_ .dat
OpenHRP.periodicCall addSignal tr.triger	

proc resettr()
-> tr.stop
-> tr.trace
-> tr.open  ${TRACE_REPOSITORY} jl_ .dat
-> tr.start
endproc
