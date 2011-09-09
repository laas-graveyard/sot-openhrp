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

#ifndef DYNAMIC_GRAPH_SOT_OPENHRP_STACK_OF_TASKS_HH
# define DYNAMIC_GRAPH_SOT_OPENHRP_STACK_OF_TASKS_HH
# include <dynamic-graph/entity.h>
# include <dynamic-graph/signal.h>
# include <dynamic-graph/signal-ptr.h>
# include <dynamic-graph/linear-algebra.h>
# include <sot/core/device.hh>
# include <sot/core/matrix-homogeneous.hh>
# include <sot/core/matrix-rotation.hh>

# include "OpenHRPCommon.hh"

class plugin_manager;

namespace dynamicgraph
{
  namespace sot
  {
    namespace openhrp
    {
      using OpenHRP::DblSequence;
      using OpenHRP::DblSequence3;
      using OpenHRP::DblSequence6;
      using OpenHRP::DblSequence9;

      class RobotState
      {
      public:
	typedef _CORBA_Unbounded_Sequence< DblSequence6 > Force;
	typedef _CORBA_Unbounded_Sequence< DblSequence9 > Attitude;
	RobotState (DblSequence& inAngle,
		    Force& inForce,
		    Attitude& inAttitude,
		   DblSequence& inTorque,
		    DblSequence3& inZmp,
		    DblSequence3& inBasePos,
		    DblSequence9& inBaseAtt)
	  : angle (inAngle),
	    force (inForce),
	    attitude (inAttitude),
	    torque (inTorque),
	    zmp(inZmp),
	    basePos(inBasePos),
	    baseAtt(inBaseAtt)
	{}

	DblSequence& angle;
	Force& force;
	Attitude& attitude;
	DblSequence& torque;
	DblSequence3& zmp;
	DblSequence3& basePos;
	DblSequence9& baseAtt;
      };

      /// \brief Dynamic graph entity communicating with the OpenHRP
      /// framework.
      ///
      /// This entity *must* be created by the custom OpenHRP plug-in.
      /// It contains setup/control/cleanup real-time methods matching
      /// the OpenHRP architecture. In particular control will trigger
      /// the recomputation of the dynamic-graph to generate a new
      /// control value.
      ///
      /// In practice, this entity takes into account the robot state
      /// during setup *ONLY*! When control is called, the control is
      /// integrated using the previous state.  The underlying reason
      /// is that the PD controller used on HRP-2 prevents any
      /// movement if the control is too small. By integrating the
      /// command, the difference between the real robot state and the
      /// wanted robot state (the set point) will increase enough to
      /// make the robot move.
      ///
      /// See also RobotSimu which provides an OpenHRP independent
      /// controller. The only difference between RobotSimu and this
      /// entity is the data forwarding between the OpenHRP framework
      /// and the dynamic-graph framework. Otherwise, both are
      /// iteratively integrating the command.
      class StackOfTasks : public Device
      {
      public:
	/// \name Generic entity methods and atttributes.
	/// \{

	/// \brief String containing the name of the class for the
	/// factory.
	static const std::string CLASS_NAME;

	/// \brief Retrieve the name of the class.
	virtual const std::string& getClassName () const
	{
	  return CLASS_NAME;
	}

	/// \}

	/// \brief Default constructor
	///
	/// \param entityName entity name in the dynamic-graph pool
	StackOfTasks(const std::string& entityName);
	virtual ~StackOfTasks();

	/// \name Inherited control methods.
	/// \{

	/// \brief Called when plug-in is started.
	/// \param rs robot state
	/// \param mc motor control
	virtual bool setup (RobotState* rs, RobotState* mc);

	/// \brief Called at each control loop.
	/// \param rs robot state
	/// \param mc motor control
	virtual void control (RobotState* rs, RobotState* mc);

	/// \brief Called when plug-in is stopped.
	/// \param rs robot state
	/// \param mc motor control
	virtual bool cleanup (RobotState* rs, RobotState* mc);

	/// \}

      protected:
	void updateRobotState (RobotState* rs);

      private:
	/// \brief Default integration step (i.e. 5ms for HRP-2).
	static const double TIMESTEP_DEFAULT;
	/// \brief Current integration step.
	double timestep_;
	/// \brief Previous robot configuration.
	Vector previousState_;

	/// \brief Robot state provided by OpenHRP.
	///
	/// This corresponds to the real encoders values and take into
	/// account the stabilization step. Therefore, this usually
	/// does *not* match the state control input signal.
	///
	dynamicgraph::Signal<ml::Vector, int> robotState_;
      };

    } // end of namespace openhrp.
  } // end of namespace sot.
} // end of namespace dynamicgraph.

#endif //! DYNAMIC_GRAPH_SOT_OPENHRP_STACK_OF_TASKS_HH
