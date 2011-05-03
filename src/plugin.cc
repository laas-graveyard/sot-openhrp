// Copyright 2010, 2011 François Bleibel, Florent Lamiraux, Thomas
// Moulard, Olivier Stasse, JRL, CNRS/AIST
//
// This file is part of sot-openhrp.
// sot-openhrp is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
//
// sot-openhrp is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// sot-openhrp. If not, see <http://www.gnu.org/licenses/>.

#include <strings.h>
#include <Python.h>

#include <dynamic-graph/exception-factory.h>
#include <dynamic-graph/debug.h>
#include <sot/core/exception-factory.hh>
#include <sot/core/debug.hh>

#include "plugin.hh"
#include "dynamic-graph/python/interpreter.hh"

const std::string SOT_OPENHRP_OUTPUT_FILE ("/tmp/sot.out");
const std::string DT_DAT_OUTPUT_FILE ("/tmp/dt.dat");

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

plugin* create_plugin(istringstream &)
{
  return new dynamicgraph::sot::openhrp::Plugin();
}

dynamicgraph::sot::openhrp::
Plugin::Plugin()
  : interpreter_(),
    entity_(new StackOfTasks("robot_device")),
    timeArray_ (),
    timeIndex_ (0),
    t0_ (),
    t1_ (),
    started_ (false)
{
  typedef void (command_receiver::*method_t) (std::istringstream&);

  // Set to zero C structures.
  bzero(timeArray_, TIME_ARRAY_SIZE * sizeof (double));
  bzero(&t0_, sizeof (timeval));
  bzero(&t1_, sizeof (timeval));

  register_method (":initialize", (method_t) &Plugin::start);
  register_method (":finalize", (method_t) &Plugin::stop);

  assigned_time = 0.005;
}

dynamicgraph::sot::openhrp::
Plugin::~Plugin()
{
  // Do not delete entity_
}

void
dynamicgraph::sot::openhrp::
Plugin::stop (std::istringstream&)
{
  // Write log data to file.
  writeLog ();
}

void
dynamicgraph::sot::openhrp::
Plugin::writeLog ()
{
  std::ofstream of (DT_DAT_OUTPUT_FILE.c_str ());
  of << "# size = " << timeIndex_ << std::endl;
  for (unsigned i = 0; i < timeIndex_; ++i)
    of << i << "\t" << timeArray_[i] << std::endl;
}

void
dynamicgraph::sot::openhrp::
Plugin::captureTime (timeval& t)
{
  gettimeofday (&t, NULL);
}

void
dynamicgraph::sot::openhrp::
Plugin::logTime (const timeval& t0, const timeval& t1)
{
  double dt =
    (t1.tv_sec - t0.tv_sec) * 1000.
    + (t1.tv_usec - t0.tv_usec + 0.) / 1000.;

  if (timeIndex_ < TIME_ARRAY_SIZE)
    timeArray_[timeIndex_++] = dt;
}

void
dynamicgraph::sot::openhrp::
Plugin::start (std::istringstream&)
{
  std::ofstream aof (SOT_OPENHRP_OUTPUT_FILE.c_str (), std::ios_base::app);
  runPython(aof, "import sys, os", interpreter_);
  runPython(aof, "pythonpath = os.environ['PYTHONPATH']", interpreter_);
  runPython(aof, "path = []", interpreter_);
  runPython(aof,
	    "for p in pythonpath.split(':'):\n"
	    "  if p not in sys.path:\n"
	    "    path.append(p)", interpreter_);
  runPython(aof, "path.extend(sys.path)", interpreter_);
  runPython(aof, "sys.path = path", interpreter_);
  runPython(aof, "from dynamic_graph.sot.openhrp.prologue import robot, solver",
	    interpreter_);
  interpreter_.startCorbaServer("openhrp", "", "stackOfTasks", "");
  started_ = true;
}

bool dynamicgraph::sot::openhrp::
Plugin::setup  (OpenHRP::RobotState* rs, OpenHRP::RobotState* mc)
{
  if (!started_)
    {
      std::cout << "Please call ':start' before starting the plug-in."
  		<< std::endl;
      return false;
    }

  // Log control loop start time.
  captureTime (t0_);

  // Initialize client to seqplay.
  dynamicgraph::sot::openhrp::RobotState robotState(rs->angle, rs->force,
						    rs->attitude, rs->torque,
						    rs->zmp, rs->basePos,
						    rs->baseAtt);

  dynamicgraph::sot::openhrp::RobotState motorCommand(mc->angle, mc->force,
						      mc->attitude, mc->torque,
						      mc->zmp, mc->basePos,
						      mc->baseAtt);

  // Log control loop end time and compute time spent.
  captureTime (t1_);
  logTime (t0_, t1_);

  return entity_->setup(&robotState, &motorCommand);
}

void dynamicgraph::sot::openhrp::
Plugin::control(OpenHRP::RobotState* rs, OpenHRP::RobotState* mc)
{
  // Log control loop start time.
  captureTime (t0_);

  dynamicgraph::sot::openhrp::RobotState robotState(rs->angle, rs->force,
						    rs->attitude, rs->torque,
						    rs->zmp, rs->basePos,
						    rs->baseAtt);

  dynamicgraph::sot::openhrp::RobotState motorCommand(mc->angle, mc->force,
						      mc->attitude, mc->torque,
						      mc->zmp, mc->basePos,
						      mc->baseAtt);

  // Log control loop end time and compute time spent.
  captureTime (t1_);
  logTime (t0_, t1_);

  sotDEBUG(25) << "dt = " << dt << std::endl;

  return entity_->control(&robotState, &motorCommand);
}

bool dynamicgraph::sot::openhrp::
Plugin::cleanup(OpenHRP::RobotState* rs, OpenHRP::RobotState* mc)
{
  // Log control loop start time.
  captureTime (t0_);

  dynamicgraph::sot::openhrp::RobotState robotState(rs->angle, rs->force,
						    rs->attitude, rs->torque,
						    rs->zmp, rs->basePos,
						    rs->baseAtt);

  dynamicgraph::sot::openhrp::RobotState motorCommand(mc->angle, mc->force,
						      mc->attitude, mc->torque,
						      mc->zmp, mc->basePos,
						      mc->baseAtt);

  // Log control loop end time and compute time spent.
  captureTime (t1_);
  logTime (t0_, t1_);

  return entity_->cleanup(&robotState, &motorCommand);
}
