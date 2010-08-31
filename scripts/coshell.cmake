loadPlugin server-abstract${DYN_LIB_EXT} ${CMAKE_INSTALL_PREFIX}/lib/plugin
loadPlugin server-command${DYN_LIB_EXT} ${CMAKE_INSTALL_PREFIX}/lib/plugin

debugtrace true
new ServerCommand coshell
coshell.initAndStart sot
