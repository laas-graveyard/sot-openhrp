# Copyright (C) 2011 Thomas Moulard, JRL, CNRS/AIST.
#
# This file is part of sot-openhrp.
# sot-openhrp is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-openhrp is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-openhrp. If not, see <http://www.gnu.org/licenses/>.

CONFIG_FILES("sot.xml")
CONFIG_FILES("sot-bush.xml")

INSTALL(
  FILES
  ${CMAKE_CURRENT_BINARY_DIR}/sot.xml
  ${CMAKE_CURRENT_BINARY_DIR}/sot-bush.xml
  DESTINATION ${GRX_PREFIX}/${ROBOT}/project
  PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ
)
