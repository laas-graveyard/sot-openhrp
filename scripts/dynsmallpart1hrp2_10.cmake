run ${CMAKE_INSTALL_PREFIX}/script/base
loadPlugin sotDynamicHrp2_10${DYN_LIB_EXT}
echo Creating the small dynamic for hrp-2 N10 small
new DynamicHrp2_10 dyn
new DynamicHrp2_10 dyn2
run ${CMAKE_INSTALL_PREFIX}/script/dynfilessmallhrp2_10

