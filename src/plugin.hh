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

#ifndef DYNAMIC_GRAPH_SOT_OPENHRP_PLUGIN_HH
# define DYNAMIC_GRAPH_SOT_OPENHRP_PLUGIN_HH
# include <sys/time.h>

# include <dynamic_graph_bridge/ros_interpreter.hh>
# include "stack-of-tasks.hh"

# include "Plugin_impl.h"

namespace dynamicgraph
{
  namespace sot
  {
    namespace openhrp
    {
      /// \brief OpenHRP plugin to control HRP2
      ///
      /// This plug-in plugs dynamic-graph into the OpenHRP
      /// architecture. This class implements a custom OpenHRP
      /// plug-in.
      ///
      /// An OpenHRP plug-in provides both CORBA requests for
      /// asynchronous/non real-time changes and a control loop
      /// executed in a real-time context.
      ///
      /// The real-time part gathers the setup, control and cleanup
      /// methods respectively called during initialization, every
      /// control loop and when the plug-in is destroyed. This plug-in
      /// is controlled by a Python script running in OpenHRP
      /// (GrxUi.sh). In Python, setup and destroy are mapped to the
      /// start and stop commands.
      ///
      /// As setup and cleanup are executed in the real time loop,
      /// additional start/stop custom procedures are provided to
      /// allow long initialization (i.e. parsing the robot model,
      /// etc). These are the start and stop methods of this class.
      /// They can be accessed in the Python script through the
      /// generic sendMsg command (commands are respectively
      /// initialize and finalize).
      ///
      /// To finish, the entity_ attribute creates the device that
      /// will be used in dynamic-graph. A device provides the robot
      /// state to the graph and receives the control at the end of
      /// the control loop. See stack-of-tasks.hh for more
      /// information.
      class Plugin : public ::plugin
      {
      public:

	explicit Plugin ();
	virtual ~Plugin ();

	/// \name Inherited control methods.
	/// \{

	/// \brief Called when plug-in is started.
	/// \param rs robot state
	/// \param mc motor control
	virtual bool setup (OpenHRP::RobotState* rs, OpenHRP::RobotState* mc);

	/// \brief Called at each control loop.
	/// \param rs robot state
	/// \param mc motor control
	virtual void control (OpenHRP::RobotState* rs, OpenHRP::RobotState* mc);

	/// \brief Called when plug-in is stopped.
	/// \param rs robot state
	/// \param mc motor control
	virtual bool cleanup (OpenHRP::RobotState* rs, OpenHRP::RobotState* mc);

	/// \}

      private:
	/// \name Custom OpenHRP commands.
	/// \{

	/// \brief Non real-time initialization method.
	///
	/// This triggers the prologue interpretation and launch the
	/// CORBA server (i.e. the one receiving Python code).
	void start (std::istringstream&);

	/// \brief Non real-time stopping method.
	///
	/// This dumps the logged information.
	void stop (std::istringstream&);
	/// \}

	/// \brief Capture the current time in the argument.
	/// \param t store current time
	void captureTime (timeval& t);

	/// \brief Log time spent between t0 and t1 in the next
	/// free case of timeArray.
	///
	/// \param t0 begin time
	/// \param t1 end time
	void logTime (const timeval& t0, const timeval& t1);

	/// \brief Write logged times in a file.
	void writeLog ();


	/// \brief Size of the array logging time spent in control loop.
	static const unsigned int TIME_ARRAY_SIZE = 100000;

	/// Embedded python interpreter accessible via ROS.
	dynamicgraph::Interpreter interpreter_;
	/// Pointer to Entity StackOfTasks
	StackOfTasks* entity_;

	/// \brief Log time spend during control loops.
	double timeArray_[TIME_ARRAY_SIZE];
	/// \brief First unfilled item in timeArray.
	unsigned int timeIndex_;

	/// \brief Timestamp matching the beginning of the control
	/// loop.
	timeval t0_;
	/// \brief Timestamp matching the end of the control loop.
	timeval t1_;

	/// \brief Did the non real-time start function been called?
	bool started_;
      };

    } // end of namespace openhrp.
  } // end of namespace sot.
} // end of namespace dynamicgraph.

#endif // DYNAMIC_GRAPH_SOT_OPENHRP_PLUGIN_HH
