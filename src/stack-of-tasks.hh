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
//#include <sot-core/vector-roll-pitch-yaw.h>
#include <sot-core/matrix-homogeneous.h>
#include <sot-core/matrix-rotation.h>

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
      enum robotType {
	hrp2_10_small,     // Control new HRP-2 10 with new HRP-2 10 small model
	hrp2_10_small_old, // Control new HRP-2 10 with old HRP-2 10 small model
	hrp2_14_small      //! Control HRP-2 14 with its small model
      };

      /* ------------------------------------------------------------------ */
      /* --- CLASS -------------------------------------------------------- */
      /* ------------------------------------------------------------------ */

      /// \brief Entity embedded into an OpenHRP plug-in to control HRP2
      ///

      class StackOfTasks : public Entity
      {
	public:  /* --- GENERIC PLUGIN IMPLEMENTATION --- */

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
	enum PluginState {
	  sotNO_STATE    = 0x00
	  ,sotJUST_BUILT = 0x01  //! Just build, waiting for init
	  ,sotINIT       = 0x02  //! Init done, waiting for setup
	  ,sotSETUP      = 0x04  //! Setup done, ready to control
	  ,sotLOOP       = 0x08  //! Setup done, already controlled once
	  ,sotHOLD       = 0x10  //! Hold requiered
	  ,sotFINISHING  = 0x20  //! Hold considered, control to finish
	  ,sotFINISHED   = 0x40  //! Finish done, waiting for cleanup
	  ,sotCLEANED_UP = 0x80  //! Cleanup done, ready for destruction
	};

	/* ! Specify the reference state. */
	enum sotReferenceState
	{
	  REFSTATE_RS = 0x00, //! The default.
	  REFSTATE_MC = 0x01  //! The conservative one.
	};

	void setNumberDofs(int numberDofs);
	void setRobotType(robotType whichRobot);

	void sotInit();
	void sotControlLoop(RobotState* rs, RobotState* mc);
	void sotControlFinishing(RobotState* rs, RobotState* mc);

	static const std::string CLASS_NAME;
	virtual const std::string& getClassName() const {
	  return CLASS_NAME;
	}

	void commandLine(const std::string& cmdLine,
			  std::istringstream& cmdArgs,
			  std::ostream& os);

	void updateMC(RobotState *mc, maal::boost::Vector &VectorCommand);
	void updateFromMC(RobotState *mc,
			  maal::boost::Vector &VectorCommand,
			  unsigned int ref=0);
	void updateTorque(RobotState* rs,
			  maal::boost::Vector &torque);

	void play();
	void pause();

      private: /* --- MEMBERS --- */
	//! Specific fields of the
	unsigned int iter_;
	maal::boost::Vector robotStatePrec_;
	bool robotStatePrecInit_;
	bool reinitFromMc_;

	maal::boost::Vector motorCommandPrec_;
	bool motorCommandInit_;

#ifdef HAVE_LIBBOOST_THREAD
	boost::try_mutex controlMutex_;
#endif
	PluginState pluginState_;
	bool suspend_;

# ifdef SOT_CHECK_TIME
	static const int TIME_ARRAY_SIZE = 100000;
	double timeArray[TIME_ARRAY_SIZE];
	int stateArray[TIME_ARRAY_SIZE];
	int timeIndex_;
# endif  // #ifdef SOT_CHECK_TIME

	static const double TIMESTEP_DEFAULT;
	double timestep_;

	/*! Used to handle different robots */
	unsigned int numberDofs_;
	robotType whichRobot_;

	void updateHRP2SmallFromMC(RobotState *mc,
				   maal::boost::Vector &VectorCommand,
				   unsigned int ref=0);
	void updateHRP2SmallMC(RobotState *mc,
			       maal::boost::Vector &VectorCommand,
			       unsigned int ref=0);
	void updateHRP2Smalltorque(RobotState *rs,
				   maal::boost::Vector &Torque);

	void updateHRP210SmallOldFromMC(RobotState *mc,
					maal::boost::Vector &VectorCommand,
					unsigned int ref=0);
	void updateHRP210SmallOldMC(RobotState *mc,
				    maal::boost::Vector &VectorCommand,
				    unsigned int ref =0);

	void updateHRP210SmallOldtorque(RobotState *rs,
					maal::boost::Vector &Torque);

	enum ForceSignalSource
	{
	  FORCE_SIGNAL_RLEG,
	  FORCE_SIGNAL_LLEG,
	  FORCE_SIGNAL_RARM,
	  FORCE_SIGNAL_LARM
	};
	bool withForceSignals[4];
	
	dynamicgraph::SignalPtr< ml::Vector,int > controlSIN;
	dynamicgraph::SignalPtr< ml::Vector,int > attitudeSIN;
	dynamicgraph::SignalPtr< ::sot::MatrixHomogeneous, int > positionSIN;
	dynamicgraph::SignalPtr< ml::Vector,int > zmpSIN;
      
	dynamicgraph::Signal< ml::Vector,int > stateSOUT;
	dynamicgraph::Signal< ml::Vector,int >* forcesSOUT[4];
	dynamicgraph:: Signal< ::sot::MatrixRotation,int > attitudeSOUT;

	dynamicgraph::Signal< ml::Vector,int> pseudoTorqueSOUT;
	bool activatePseudoTorqueSignal;
	//Signal<ml::Vector,int> velocitySOUT;
	//bool activateVelocitySignal;
	dynamicgraph::Signal<ml::Vector,int> previousStateSOUT;
	dynamicgraph::Signal<ml::Vector,int> previousControlSOUT;
	/*! \brief The current state of the robot from the command viewpoint. */
	dynamicgraph::Signal<ml::Vector,int> motorcontrolSOUT;
	/*! \brief The ZMP reference send by the previous controller. */
	dynamicgraph::Signal<ml::Vector,int> ZMPPreviousControllerSOUT;
	bool activatePreviousControlSignal;

	/*! Member storing the reference state.  */
	unsigned int referenceState_;

      }; // class StackOfTasks

    } // namespace openhrp
  } // namespace sot
} //namespace dynamicgraph

#endif //DYNAMIC_GRAPH_SOT_OPENHRP_STACK_OF_TASKS_HH
