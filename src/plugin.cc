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
#include <dynamic_graph_bridge/ros_init.hh>

#include "plugin.hh"
#include "dynamic-graph/python/interpreter.hh"

const std::string SOT_OPENHRP_OUTPUT_FILE ("/tmp/sot.out");
const std::string DT_DAT_OUTPUT_FILE ("/tmp/dt.dat");

plugin* create_plugin (istringstream &)
{
  return new dynamicgraph::sot::openhrp::Plugin ();
}

namespace dynamicgraph
{
  namespace sot
  {
    namespace openhrp
    {
      static void
      runPython (std::ostream& file,
		 const std::string& command,
		 ::dynamicgraph::Interpreter& interpreter)
      {
	file << ">>> " << command << std::endl;
	std::string value = interpreter.runCommand (command);
	if (value != "None")
	  file << value;
      }

      Plugin::Plugin()
	: interpreter_ (dynamicgraph::rosInit (false)),
	  entity_ (new StackOfTasks ("robot_device")),
	  timeArray_ (),
	  timeIndex_ (0),
	  t0_ (),
	  t1_ (),
	  started_ (false)
      {
	typedef void (command_receiver::*method_t) (std::istringstream&);

	// Set to zero C structures.
	bzero (timeArray_, TIME_ARRAY_SIZE * sizeof (double));
	bzero (&t0_, sizeof (timeval));
	bzero (&t1_, sizeof (timeval));

	register_method (":initialize", (method_t) &Plugin::start);
	register_method (":finalize", (method_t) &Plugin::stop);

	assigned_time = 0.005;
      }

      Plugin::~Plugin ()
      {
	// Do not delete entity_
      }

      void
      Plugin::stop (std::istringstream&)
      {
	// Write log data to file.
	writeLog ();
      }

      void
      Plugin::writeLog ()
      {
	std::ofstream of (DT_DAT_OUTPUT_FILE.c_str ());
	of << "# size = " << timeIndex_ << std::endl;
	for (unsigned i = 0; i < timeIndex_; ++i)
	  of << i << "\t" << timeArray_[i] << std::endl;
      }

      void
      Plugin::captureTime (timeval& t)
      {
	gettimeofday (&t, NULL);
      }

      void
      Plugin::logTime (const timeval& t0, const timeval& t1)
      {
	double dt =
	  (t1.tv_sec - t0.tv_sec) * 1000.
	  + (t1.tv_usec - t0.tv_usec + 0.) / 1000.;

	if (timeIndex_ < TIME_ARRAY_SIZE)
	  timeArray_[timeIndex_++] = dt;
	sotDEBUG(25) << "dt = " << dt << std::endl;
      }

      void
      Plugin::start (std::istringstream&)
      {
	std::ofstream aof (SOT_OPENHRP_OUTPUT_FILE.c_str ());
	runPython (aof, "import sys, os", interpreter_);
	runPython (aof, "pythonpath = os.environ['PYTHONPATH']", interpreter_);
	runPython (aof, "path = []", interpreter_);
	runPython (aof,
		   "for p in pythonpath.split(':'):\n"
		   "  if p not in sys.path:\n"
		   "    path.append(p)", interpreter_);
	runPython (aof, "path.extend(sys.path)", interpreter_);
	runPython (aof, "sys.path = path", interpreter_);
	runPython
	  (aof,
	   "from dynamic_graph.sot.openhrp.prologue import robot, solver",
	   interpreter_);

	// Calling again rosInit here to start the spinner. It will
	// deal with topics and services callbacks in a separate, non
	// real-time thread. See roscpp documentation for more
	// information.
	dynamicgraph::rosInit (true);
	started_ = true;
      }

      bool
      Plugin::setup (OpenHRP::RobotState* rs, OpenHRP::RobotState* mc)
      {
	if (!started_)
	  {
	    std::cout
	      << "Please call ':initialize' before starting the plug-in."
	      << std::endl;
	    return false;
	  }

	// Log control loop start time.
	captureTime (t0_);

	// Initialize client to seqplay.
	RobotState robotState (rs->angle, rs->force,
			       rs->attitude, rs->torque,
			       rs->zmp, rs->basePos,
			       rs->baseAtt);

	RobotState motorCommand (mc->angle, mc->force,
				 mc->attitude, mc->torque,
				 mc->zmp, mc->basePos,
				 mc->baseAtt);
	bool res=false;
	try
	  {
	    res = entity_->setup (&robotState, &motorCommand);
	  } catch SOT_RETHROW;

	// Log control loop end time and compute time spent.
	captureTime (t1_);
	logTime (t0_, t1_);

	return res;
      }

      void
      Plugin::control (OpenHRP::RobotState* rs, OpenHRP::RobotState* mc)
      {
	// Log control loop start time.
	captureTime (t0_);

	RobotState robotState (rs->angle, rs->force,
			       rs->attitude, rs->torque,
			       rs->zmp, rs->basePos,
			       rs->baseAtt);

	RobotState motorCommand (mc->angle, mc->force,
				 mc->attitude, mc->torque,
				 mc->zmp, mc->basePos,
				 mc->baseAtt);
	try 
	  {
	    entity_->control (&robotState, &motorCommand);
	  } catch SOT_RETHROW;

	// Log control loop end time and compute time spent.
	captureTime (t1_);
	logTime (t0_, t1_);
      }

      bool
      Plugin::cleanup (OpenHRP::RobotState* rs, OpenHRP::RobotState* mc)
      {
	// Log control loop start time.
	captureTime (t0_);

	RobotState robotState(rs->angle, rs->force,
			      rs->attitude, rs->torque,
			      rs->zmp, rs->basePos,
			      rs->baseAtt);

	RobotState motorCommand(mc->angle, mc->force,
				mc->attitude, mc->torque,
				mc->zmp, mc->basePos,
				mc->baseAtt);
	bool res = false;
	try
	  {
	    res = entity_->cleanup (&robotState, &motorCommand);
	  } catch SOT_RETHROW;

	// Log control loop end time and compute time spent.
	captureTime (t1_);
	logTime (t0_, t1_);

	return res;
      }

    } // namespace openhrp
  } // namespace sot
} // namespace dynamicgraph
