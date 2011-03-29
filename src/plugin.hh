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
#define DYNAMIC_GRAPH_SOT_OPENHRP_PLUGIN_HH

#include <dynamic-graph/corba/interpreter.hh>
#include "stack-of-tasks.hh"

#include "Plugin_impl.h"
#define SOT_CHECK_TIME

namespace dynamicgraph {
  namespace sot {
    namespace openhrp {

      /// \brief OpenHRP plugin to control HRP2
      class Plugin : public ::plugin
      {
      public:
	Plugin();
	virtual ~Plugin();
	/** @name GenericPlugin
	 * Generic plugin implementation: call StackOfStack methods
	 */
	// @{
	virtual bool setup  (OpenHRP::RobotState* rs, OpenHRP::RobotState* mc);
	virtual void control(OpenHRP::RobotState* rs, OpenHRP::RobotState* mc);
	virtual bool cleanup(OpenHRP::RobotState* rs, OpenHRP::RobotState* mc);
	// @}
	
      private:
	/// Embedded python interpreter accessible via Corba
	dynamicgraph::corba::Interpreter interpreter_;
	/// Pointer to Entity StackOfTasks
	StackOfTasks* entity_;
# ifdef SOT_CHECK_TIME 
	unsigned int timeIndex;
#endif // #ifdef SOT_CHECK_TIME 

      };
    } // namespace openhrp
  } // namespace sot
} //namespace dynamicgraph

#endif //DYNAMIC_GRAPH_SOT_OPENHRP_PLUGIN_HH
