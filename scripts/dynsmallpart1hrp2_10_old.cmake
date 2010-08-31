run ${CMAKE_INSTALL_PREFIX}/script/base
loadPlugin sotDynamicHrp2_10_old${DYN_LIB_EXT}
echo Creating the small dynamic for hrp-2 N10 small old
new DynamicHrp2_10_old dyn
new DynamicHrp2_10_old dyn2
run ${CMAKE_INSTALL_PREFIX}/script/dynfilessmallhrp2_10_old
