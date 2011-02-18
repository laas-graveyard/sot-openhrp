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
#include <dynamic-graph/debug.h>
#include <sot/core/exception-factory.hh>
#include <sot/core/debug.hh>

#include "plugin.hh"
#include "dynamic-graph/python/interpreter.hh"

#define SOT_OPENHRP_OUTPUT_FILE "/tmp/sot.out"

using dynamicgraph::sot::openhrp::Plugin;

namespace dynamicgraph {
  namespace sot {
    namespace openhrp {
      static void runPython(std::ostream& file, const std::string& command,
			    corba::Interpreter& interpreter) {
	file << ">>> " << command << std::endl;
	std::string value = interpreter.python(command);
	if (value != "None")
	  file << value;
      }
    } // namespace openhrp
  } // namespace sot
} // namespace dynamicgraph

plugin* create_plugin(istringstream &strm)
{
  return new dynamicgraph::sot::openhrp::Plugin();
}

dynamicgraph::sot::openhrp::Plugin::
Plugin() : interpreter_(), entity_(new StackOfTasks("device"))
{
  ofstream aof;
  aof.open(SOT_OPENHRP_OUTPUT_FILE, std::ios_base::app);
  runPython(aof, "import sys, os", interpreter_);
  runPython(aof, "pythonpath = ''", interpreter_);
  runPython(aof,
	    "with open('./python-path') as f:\n"
	    "  pythonpath = f.readline().rstrip('\\n ')", interpreter_);
  runPython(aof, "path = []", interpreter_);
  runPython(aof,
	    "for p in pythonpath.split(':'):\n"
	    "  if p not in sys.path:\n"
	    "    path.append(p)", interpreter_);
  runPython(aof, "path.extend(sys.path)", interpreter_);
  runPython(aof, "sys.path = path", interpreter_);
  runPython(aof, "from dynamic_graph.sot.openhrp.prologue import robot, solver",
	    interpreter_);
  assigned_time = 0.005;
  interpreter_.startCorbaServer("openhrp", "", "stackOfTasks", "");
  aof.close();
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
