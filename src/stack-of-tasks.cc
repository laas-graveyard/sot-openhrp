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

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>

#include <dynamic-graph/debug.h>
#include <sot/core/debug.hh>
#include <sot/core/exception-factory.hh>

#include "stack-of-tasks.hh"

#include "Plugin_impl.h"

using dynamicgraph::sot::openhrp::StackOfTasks;
using dynamicgraph::sot::ExceptionFactory;

const double StackOfTasks::TIMESTEP_DEFAULT = 0.005;

const std::string StackOfTasks::CLASS_NAME = "Device";

StackOfTasks::StackOfTasks(const std::string& inName)
  : dynamicgraph::sot::Device(inName),
# ifdef SOT_CHECK_TIME
    timeIndex_(0),
#endif // #ifdef SOT_CHECK_TIME
    timestep_(TIMESTEP_DEFAULT),
    previousState_()
{
}

bool StackOfTasks::setup(RobotState*, RobotState* mc)
{
#ifdef VP_DEBUG
  dynamicgraph::DebugTrace::openFile();
  dynamicgraph::sot::DebugTrace::openFile();
#endif
  // Read state from motor command
  maal::boost::Vector state = stateSOUT.access(0);

  sotDEBUG(25) << "stateSOUT.access(0) = " << state << std::endl;
  sotDEBUG(25) << "state.size() = " << state.size() << std::endl;
  sotDEBUG(25) << "mc->angle.length() = " << mc->angle.length() << std::endl;

  for (unsigned int i=6; i<state.size(); i++) {
    state(i) = mc->angle[i-6];
  }
  previousState_ = state;
  sotDEBUG(25) << "state = " << state << std::endl;
  stateSOUT.setConstant(state);
  return true;
}

void StackOfTasks::
control(RobotState* rs, RobotState* mc)
{
  // Integrate control
  increment(timestep_);
  sotDEBUG(25) << "state = " << state_ << std::endl;
  sotDEBUG(25) << "diff  = " << state_ - previousState_ << std::endl;
  previousState_ = state_;
  // Write new state into motor command
  for (unsigned int i=6; i<state_.size(); i++) {
    mc->angle[i-6] = state_(i);
  }
}

bool StackOfTasks::
cleanup(RobotState*, RobotState*)
{
  return true;
}

StackOfTasks::~StackOfTasks()
{
}
