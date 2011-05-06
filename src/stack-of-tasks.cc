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

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/debug.h>
#include <sot/core/debug.hh>
#include <sot/core/exception-factory.hh>
#include <dynamic-graph/all-commands.h>

#include "stack-of-tasks.hh"

#include "Plugin_impl.h"
#include <dynamic-graph/all-commands.h>
#include "sot/core/api.hh"

using dynamicgraph::sot::openhrp::StackOfTasks;
using dynamicgraph::sot::ExceptionFactory;

const double StackOfTasks::TIMESTEP_DEFAULT = 0.005;
const std::string StackOfTasks::CLASS_NAME = "Device";

StackOfTasks::StackOfTasks (const std::string& entityName)
  : dynamicgraph::sot::Device (entityName),
    timestep_ (TIMESTEP_DEFAULT),
    previousState_ ()
{
  for (unsigned i = 0; i < 4; ++i)
    withForceSignals[i] = true;
}

bool
StackOfTasks::setup (RobotState* rs, RobotState* mc)
{
#ifdef VP_DEBUG
  dynamicgraph::DebugTrace::openFile();
  dynamicgraph::sot::DebugTrace::openFile();
#endif

  // Read state from motor command
  int t = stateSOUT.getTime () + 1;
  maal::boost::Vector state = stateSOUT.access (t);

  sotDEBUG (25) << "stateSOUT.access (" << t << ") = " << state << std::endl;
  sotDEBUG (25) << "state.size () = " << state.size () << std::endl;
  sotDEBUG (25) << "mc->angle.length () = " << mc->angle.length () << std::endl;

  for (unsigned int i = 6; i < state.size (); ++i)
    state (i) = mc->angle[i - 6];

  previousState_ = state;
  sotDEBUG (25) << "state = " << state << std::endl;
  stateSOUT.setConstant (state);

  ml::Vector mlforces (6);
  for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 6; ++j)
	mlforces(j) = rs->force[i][j];
      forcesSOUT[i]->setConstant (mlforces);
    }

  return true;
}

void
StackOfTasks::control (RobotState* rs, RobotState* mc)
{
  // Integrate control
  increment (timestep_);
  sotDEBUG (25) << "state = " << state_ << std::endl;
  sotDEBUG (25) << "diff  = " << state_ - previousState_ << std::endl;
  previousState_ = state_;

  // Write new state into motor command
  for (unsigned int i = 6; i < state_.size (); ++i)
    mc->angle[i - 6] = state_(i);

  // Read zmp reference from input signal if plugged
  int time = controlSIN.getTime ();
  zmpSIN.recompute (time + 1);
  // Express ZMP in free flyer reference frame
  Vector zmpGlobal (4);
  for (unsigned int i = 0; i < 3; ++i)
    zmpGlobal(i) = zmpSIN(time + 1)(i);
  zmpGlobal(3) = 1.;
  MatrixHomogeneous inversePose;
  freeFlyerPose().inverse(inversePose);
  Vector localZmp = inversePose * zmpGlobal;

  ml::Vector mlforces (6);
  for (int i = 0; i < 4; ++i)
  {
    for( int j=0;j<6;++j )
      mlforces(j) = rs->force[i][j];
    forcesSOUT[i]->setConstant (mlforces);
    forcesSOUT[i]->setTime (time + 1);
  }

  for (unsigned int i = 0; i < mc->zmp.length (); ++i)
    mc->zmp[i] = localZmp(i);

  sotDEBUG (25) << "local zmp = " << localZmp << std::endl;
  sotDEBUG (10) << "mc->zmp = (" <<
    mc->zmp[0] << "," <<
    mc->zmp[1] << "," <<
    mc->zmp[2] << ")" << std::endl;

  sotDEBUG (10) << "global zmp = (" << zmpGlobal << std::endl;

  // Update position of freeflyer in global frame
  for (int i = 0;i < 3; ++i)
    mc->basePos[i] = freeFlyerPose () (i, 3);
  for(unsigned i = 0;i < 3; ++i)
    for(unsigned j = 0; j < 3; ++j)
      mc->baseAtt[i * 3 + j] = freeFlyerPose () (i, j);
}

bool
StackOfTasks::cleanup (RobotState*, RobotState*)
{
  return true;
}

StackOfTasks::~StackOfTasks()
{
}
