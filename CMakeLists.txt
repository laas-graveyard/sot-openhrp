# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST
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

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/lapack.cmake)
INCLUDE(cmake/cpack.cmake)

SET(PROJECT_NAME sot-openhrp)
SET(PROJECT_DESCRIPTION "jrl-walkgen bindings for dynamic-graph.")
SET(PROJECT_URL "https://github.com/jrl-umi3218/sot-openhrp")

SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")

# Define the location of the CMakeModules directory.
SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/CMakeModules)

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)

SET(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

SETUP_PROJECT()
string(REPLACE "-pedantic" "" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})

OPTION(INSTALL_IN_OPENHRP "Install plugin in OpenHRP" ON)

SET(BOOST_COMPONENTS thread)
SEARCH_FOR_BOOST()
ADD_DEFINITIONS(-DHAVE_LIBBOOST_THREAD)

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("omniORB4 >= 4.1.1")
ADD_REQUIRED_DEPENDENCY("jrl-walkgen >= 3.0.0")
ADD_REQUIRED_DEPENDENCY("jrl-mal >= 1.8.0")

ADD_REQUIRED_DEPENDENCY("sot-core >= 1.0")

# Search for closed-source packages.
ADD_REQUIRED_DEPENDENCY("hrp2-10 >= 1.0.0")

# Search for OpenHRP.
INCLUDE(${CMAKE_MODULE_PATH}/UseOmniORB4.cmake)
FIND_PACKAGE(OpenHRP REQUIRED)

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(include)
#ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()