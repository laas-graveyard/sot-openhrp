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

#ifndef DYNAMIC_GRAPH_SOT_OPENHRP_PLUGIN_HH
# define DYNAMIC_GRAPH_SOT_OPENHRP_PLUGIN_HH
# include <sys/time.h>

# include <dynamic-graph/corba/interpreter.hh>
# include "stack-of-tasks.hh"

# include "Plugin_impl.h"

namespace dynamicgraph {
  namespace sot {
    namespace openhrp {

      /// \brief OpenHRP plugin to control HRP2
      class Plugin : public ::plugin
      {
      public:
	Plugin();
	virtual ~Plugin();
	/// \name Inherited control methods.
	/// \{

	/// \brief Called when plug-in is started.
	virtual bool setup  (OpenHRP::RobotState* rs, OpenHRP::RobotState* mc);

	/// \brief Called at each control loop.
	virtual void control(OpenHRP::RobotState* rs, OpenHRP::RobotState* mc);

	/// \brief Called when plug-in is stopped.
	virtual bool cleanup(OpenHRP::RobotState* rs, OpenHRP::RobotState* mc);

	/// \}
	
      private:
	void dumpLog (std::istringstream&);

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

	/// Embedded python interpreter accessible via Corba
	dynamicgraph::corba::Interpreter interpreter_;
	/// Pointer to Entity StackOfTasks
	StackOfTasks* entity_;

	/// \brief Log time spend during control loops.
	double timeArray_[TIME_ARRAY_SIZE];
	/// \brief First unfilled item in timeArray.
	unsigned int timeIndex_;
	timeval t0_;
	timeval t1_;
      };
    } // namespace openhrp
  } // namespace sot
} //namespace dynamicgraph

#endif //DYNAMIC_GRAPH_SOT_OPENHRP_PLUGIN_HH
