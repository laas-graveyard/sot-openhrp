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

#ifndef DYNAMIC_GRAPH_SOT_OPENHRP_STACK_OF_TASKS_HH
#define DYNAMIC_GRAPH_SOT_OPENHRP_STACK_OF_TASKS_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/device.hh>
#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-rotation.hh>

#include "OpenHRPCommon.hh"

class plugin_manager;

namespace dynamicgraph {
  namespace sot {
    namespace openhrp {
      using OpenHRP::DblSequence;
      using OpenHRP::DblSequence3;
      using OpenHRP::DblSequence6;
      using OpenHRP::DblSequence9;

      class RobotState
      {
      public:
	typedef _CORBA_Unbounded_Sequence< DblSequence6 > Force;
	typedef _CORBA_Unbounded_Sequence< DblSequence9 > Attitude;
	RobotState(DblSequence& inAngle, Force& inForce, Attitude& inAttitude,
		   DblSequence& inTorque, DblSequence3& inZmp,
		   DblSequence3& inBasePos, DblSequence9& inBaseAtt) :
	  angle(inAngle), force(inForce), attitude(inAttitude),torque(inTorque),
	  zmp(inZmp), basePos(inBasePos), baseAtt(inBaseAtt)
	{}
	DblSequence& angle;
	Force& force;
	Attitude& attitude;
	DblSequence& torque;
	DblSequence3& zmp;
	DblSequence3& basePos;
	DblSequence9& baseAtt;
      };

      /* ------------------------------------------------------------------ */
      /* --- CLASS -------------------------------------------------------- */
      /* ------------------------------------------------------------------ */

      /// \brief Entity embedded into an OpenHRP plug-in to control HRP2
      ///

      class StackOfTasks : public Device
      {
	public:  /* --- GENERIC PLUGIN IMPLEMENTATION --- */

	static const std::string CLASS_NAME;
	virtual const std::string& getClassName() const {
	  return CLASS_NAME;
	}

	/** @name GenericPlugin
	 * Generic plugin implementation: implements methods inherited from
	 * the plugin class.
	 */
	// @{
	StackOfTasks(const std::string& inName);
	virtual ~StackOfTasks();
	bool setup  (RobotState* rs, RobotState* mc);
	void control(RobotState* rs, RobotState* mc);
	bool cleanup(RobotState* rs, RobotState* mc);
	// @}

      private: /* --- MEMBERS --- */
	//! Specific fields of the
	static const double TIMESTEP_DEFAULT;
	double timestep_;
	Vector previousState_;
      }; // class StackOfTasks
    } // namespace openhrp
  } // namespace sot
} //namespace dynamicgraph

#endif //DYNAMIC_GRAPH_SOT_OPENHRP_STACK_OF_TASKS_HH
