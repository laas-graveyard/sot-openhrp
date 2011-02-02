/*
 * Copyright 2011,
 * François Bleibel,
 * Olivier Stasse,
 * Florent Lamiraux
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

#include <Python.h>

#include <dynamic-graph/exception-factory.h>
#include <sot-core/exception-factory.h>
#include <sot-core/debug.h>

#include "plugin.hh"
#include "dynamic-graph/python/interpreter.hh"

using dynamicgraph::sot::openhrp::robotType;
using dynamicgraph::sot::openhrp::Plugin;
using dynamicgraph::sot::openhrp::hrp2_14_small;
using dynamicgraph::sot::openhrp::hrp2_10_small_old;
using dynamicgraph::sot::openhrp::hrp2_10_small;

plugin* create_plugin(istringstream &strm) 
{
  unsigned int lnbDofs = 46;
  robotType aRobotToControl = hrp2_14_small;
  string lRobotName;
  strm >> lRobotName;

  if (lRobotName == "HRP2JRL10SmallOld")
    {
      lnbDofs = 46;
      aRobotToControl = hrp2_10_small_old;
    }
  else if (lRobotName == "HRP2JRL10Small")
    {
      lnbDofs = 48;
      aRobotToControl = hrp2_10_small;
    }
  ofstream aof;
  aof.open("/tmp/starting.txt");
  aof << "Robot name: " << lRobotName <<endl;
  aof << "NbDofs:" << lnbDofs << endl;
  aof << "aRobotToControl:" << aRobotToControl << endl;
  aof.close();
  return new dynamicgraph::sot::openhrp::Plugin(lnbDofs,aRobotToControl);
}

dynamicgraph::sot::openhrp::Plugin::
Plugin(int nbDofs, robotType robotToControl) :
  interpreter_(), entity_(NULL), pluginManager_(manager)
{
  // Create an instance of Robot in python domain
  interpreter_.python("import dynamic_graph.sot.openhrp");
  interpreter_.python
    ("robotController = dynamic_graph.sot.openhrp.Robot('robot')");
  interpreter_.python("_robotControllerObj = robotController.obj");
  // Get pointer to Robot entity
  PyObject* dict = interpreter_.local().globals();
  PyObject* robotControllerObj =
    PyDict_GetItemString(dict, "_robotControllerObj");
  if (!PyCObject_Check(robotControllerObj)) {
    throw ExceptionFactory(ExceptionFactory::GENERIC,
			   "failed to get pointer to entity Robot");
  }
  entity_ = static_cast<StackOfTasks*>(PyCObject_AsVoidPtr(robotControllerObj));
  entity_->setNumberDofs(nbDofs);
  entity_->setRobotType(robotToControl);

  assigned_time = 0.005;
}

dynamicgraph::sot::openhrp::
Plugin::~Plugin()
{
  // Do not delete entity_
}

bool dynamicgraph::sot::openhrp::
Plugin::setup  (OpenHRP::RobotState* rs, OpenHRP::RobotState* mc)
{
  // Initialize client to seqplay.
  sotDEBUGIN(5);
  if (pluginManager_ == NULL)
    throw ::sot::ExceptionFactory(::sot::ExceptionFactory::GENERIC,
				  "OpenHRP plugin manager has not been set");
  seqplayClient_ = SequencePlayer::_narrow(pluginManager_->find("seq",""));
  sotDEBUG(25) << "Seqplugin: " << seqplayClient_ <<endl;

  dynamicgraph::sot::openhrp::RobotState robotState(rs->angle, rs->force,
						    rs->attitude, rs->torque,
						    rs->zmp, rs->basePos,
						    rs->baseAtt);

  dynamicgraph::sot::openhrp::RobotState motorCommand(mc->angle, mc->force,
						      mc->attitude, mc->torque,
						      mc->zmp, mc->basePos,
						      mc->baseAtt);
  return entity_->setup(&robotState, &motorCommand);
}

void dynamicgraph::sot::openhrp::
Plugin::control(OpenHRP::RobotState* rs, OpenHRP::RobotState* mc)
{
  dynamicgraph::sot::openhrp::RobotState robotState(rs->angle, rs->force,
						    rs->attitude, rs->torque,
						    rs->zmp, rs->basePos,
						    rs->baseAtt);

  dynamicgraph::sot::openhrp::RobotState motorCommand(mc->angle, mc->force,
						      mc->attitude, mc->torque,
						      mc->zmp, mc->basePos,
						      mc->baseAtt);
  return entity_->control(&robotState, &motorCommand);
}

bool dynamicgraph::sot::openhrp::
Plugin::cleanup(OpenHRP::RobotState* rs, OpenHRP::RobotState* mc)
{
  dynamicgraph::sot::openhrp::RobotState robotState(rs->angle, rs->force,
						    rs->attitude, rs->torque,
						    rs->zmp, rs->basePos,
						    rs->baseAtt);

  dynamicgraph::sot::openhrp::RobotState motorCommand(mc->angle, mc->force,
						      mc->attitude, mc->torque,
						      mc->zmp, mc->basePos,
						      mc->baseAtt);
  return entity_->cleanup(&robotState, &motorCommand);
}
