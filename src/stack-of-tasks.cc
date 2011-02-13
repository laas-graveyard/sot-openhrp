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

#include <sot-core/debug.h>
#include <sot-core/exception-factory.h>

#include "stack-of-tasks.hh"

#include "Plugin_impl.h"

#define SOT_OPENHRP_OUTPUT_FILE "/tmp/sot.out"

using dynamicgraph::sot::openhrp::StackOfTasks;
const double StackOfTasks::TIMESTEP_DEFAULT = 0.005;

const std::string StackOfTasks::CLASS_NAME = "Device";

StackOfTasks::StackOfTasks(const std::string& inName)
  : ::dynamicgraph::Entity(inName),
   iter_(0),
   robotStatePrec_((unsigned int)0),
# ifdef SOT_CHECK_TIME
   timeIndex_(0),
#endif // #ifdef SOT_CHECK_TIME
   timestep_(TIMESTEP_DEFAULT),
   controlSIN(NULL,"StackOfStasks("+inName+")::input(vector)::control"),
   attitudeSIN(NULL,"StackOfStasks("+inName+")::input(vector3)::attitudeIN"),
   positionSIN
   (NULL,"StackOfStasks("+inName+")::input(sotMatrixHomogeneous)::positionIN"),
   zmpSIN(NULL,"StackOfStasks("+inName+")::input(vector3)::zmp"),
   stateSOUT("StackOfStasks("+inName+")::output(vector)::state"),
   attitudeSOUT
   ("StackOfStasks("+inName+")::output(sotMatrixRotation)::attitude"),

   pseudoTorqueSOUT("StackOfStasks("+inName+")::output(vector)::ptorque"),
   activatePseudoTorqueSignal(false),
   previousStateSOUT
   ("(StackOfStasks"+inName+")::output(vector)::previousState"),
   previousControlSOUT
   ("StackOfStasks("+inName+")::output(vector)::previouscontrol"),
   motorcontrolSOUT("StackOfStasks("+inName+")::output(vector)::motorcontrol"),
   ZMPPreviousControllerSOUT
   ("StackOfStasks("+inName+")::output(vector)::zmppreviouscontroller")
{
  /* --- FORCES --- */
  for(int i=0;i<4;++i){ withForceSignals[i] = false; }
  forcesSOUT[0] =
    new Signal<ml::Vector, int>("StackOfStasks("+inName+")::output(vector6)::forceRLEG");
  forcesSOUT[1] =
    new Signal<ml::Vector, int>("StackOfStasks("+inName+")::output(vector6)::forceLLEG");
  forcesSOUT[2] =
    new Signal<ml::Vector, int>("StackOfStasks("+inName+")::output(vector6)::forceRARM");
  forcesSOUT[3] =
    new Signal<ml::Vector, int>("StackOfStasks("+inName+")::output(vector6)::forceLARM");

  signalRegistration(controlSIN
		      << attitudeSIN
		      << positionSIN
		      << zmpSIN
		      << stateSOUT
		      << motorcontrolSOUT
		      << ZMPPreviousControllerSOUT
		      << *forcesSOUT[0]
		      << *forcesSOUT[1]
		      << *forcesSOUT[2]
		      << *forcesSOUT[3]
		      << attitudeSOUT
		      //		      <<velocitySOUT
		      << pseudoTorqueSOUT
		      << previousStateSOUT
		      << previousControlSOUT);

  using namespace dynamicgraph::command;
  std::string docstring;
  /* Command setStateSize. */
  docstring =
    "\n"
    "    Set size of state vector\n"
    "\n";
  addCommand("resize",
	     new Setter<StackOfTasks, unsigned int>
	     (*this, &StackOfTasks::setStateSize, docstring));
  /* Command set. */
  docstring =
    "\n"
    "    Set state vector value\n"
    "\n";
  addCommand("set",
	     new Setter<StackOfTasks, Vector>
	     (*this, &StackOfTasks::setState, docstring));
}

bool StackOfTasks::setup(RobotState*, RobotState* mc)
{
  ofstream aof;
  aof.open(SOT_OPENHRP_OUTPUT_FILE, std::ios_base::app);
  // Read state from motor command
  maal::boost::Vector state = stateSOUT.access(0);

  aof << "stateSOUT.access(0) = " << state << std::endl;
  aof << "numberDofs_ = " << numberDofs_ << std::endl;
  aof << "mc->angle.length() = " << mc->angle.length() << std::endl;

  for (unsigned int i=6; i<state.size(); i++) {
    state(i) = mc->angle[i-6];
  }

  aof << "state = " << state << std::endl;

  sotDEBUG(25) << "state = "<< state << std::endl;
  stateSOUT.setConstant(state);
  return true;
}

void StackOfTasks::
control(RobotState* rs, RobotState* mc)
{
  ofstream aof;
  aof.open(SOT_OPENHRP_OUTPUT_FILE, std::ios_base::app);

  for (unsigned int i=0; i<rs->angle.length(); i++) {
    mc->angle[i] = rs->angle[i];
  }
  return;
}

bool StackOfTasks::
cleanup(RobotState*, RobotState*)
{
  return true;
}

StackOfTasks::~StackOfTasks()
{
}

void StackOfTasks::setState(const ml::Vector& inState)
{
  numberDofs_ = inState.size();
  stateSOUT.setConstant(inState);
  motorcontrolSOUT.setConstant(inState);
}

void StackOfTasks::setStateSize(const unsigned int& numberDofs)
{
  if (numberDofs < 6) {
    std::ostringstream oss;
    oss << "dimension of robot is "
	<< numberDofs
	<< ", should be 6 or more.";
    throw(::sot::ExceptionFactory(::sot::ExceptionFactory::GENERIC, oss.str()));
  }
  numberDofs_ = numberDofs;
  robotStatePrec_ = ml::Vector((unsigned int)numberDofs-6);
  stateSOUT.setConstant(ml::Vector((unsigned int)numberDofs));
  motorCommandPrec_ = maal::boost::Vector((unsigned int)numberDofs-6);
}

