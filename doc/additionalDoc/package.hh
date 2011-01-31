/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 * This file is part of sot-openhrp.
 * sot-openhrp is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-openhrp is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-openhrp.  If not, see <http://www.gnu.org/licenses/>.
 */


/**
\mainpage

\section intro Introduction

This package builds a plugin for OpenHRP that gives access to the stack of tasks
framework within. Since the main plugin class is also a dynamicgraph::Entity,
it is accessible from inside the dynamic-graph framework as any other entity.

\section req Requirements
This package has the following dependencies:
\li sot-core
\li jrl-mal
\li hrp2Dynamics [optional]
\li hrp2-10-small [optional]

To download and install these packages, please visit https://github.com/jrl-umi3218.

\section overview API overview
As most packages based on the dynamic-graph framework (see https://github.com/jrl-umi3218/dynamic-graph),
the functionality is exposed through entities. Hence .so or .dll (dynamic-link) libraries are
generated in the dynamic-graph plugins directory.

The following entities are created by this package:\n
(all entites are placed in the namespace sot::)
\li sot::StackOfTasks

See each entity's documentation page for more information (when available).

*/
