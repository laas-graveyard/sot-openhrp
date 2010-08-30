/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright JRL-Japan, 2010
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      StackOfTasks.h
 * Project:   SOT
 * Author:    Olivier Stasse
 *            Nicolas Mansard
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef __STACK_OF_TASKS_OPENHRP_PLUGIN_H__
#define __STACK_OF_TASKS_OPENHRP_PLUGIN_H__

/* -------------------------------------------------------------------------- */
/* --- INCLUDE -------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/* --- HRP --- */

#ifdef OPENHRP_VERSION_2
#include "plugin.h"
#include "dynamicsPlugin.h"
#include "seqplugin.h"

typedef robot_state sotRobotState;
typedef motor_command sotMotorCommand;
typedef seqplugin_ptr sotSequencePlayer_var;
typedef seqplugin sotSequencePlayer;

#define ROBOT_STATE_VAR_POS(x) x->waistPos
#define ROBOT_STATE_VAR_ATT(x) x->waistRpy

#endif

#ifdef OPENHRP_VERSION_3
#include <MatrixAbstractLayer/boost.h>
#include "Plugin_impl.h"
#include "SequencePlayer.h"
/*! Include for OpenHRP new dynamics library. */
#include "Body.h"
#include "Link.h"
#include "LinkTraverse.h"
#include "ModelLoaderUtil.h"

typedef RobotState sotRobotState;
typedef RobotState sotMotorCommand;
typedef SequencePlayer_var sotSequencePlayer_var;
typedef SequencePlayer sotSequencePlayer;

#define ROBOT_STATE_VAR_POS(x) x->basePos
#define ROBOT_STATE_VAR_ATT(x) x->baseAtt

#endif

#include "bodyinfo.h"	    


#include <sot/sotInterpretor.h>
#include <sot/sotPluginLoader.h>
#include <sot/sotEntity.h>
#include <sot/sotSignal.h>
#include <sot/sotSignalPtr.h>
#include <sot/sotVectorRollPitchYaw.h>

#include <sot/sotPeriodicCall.h>

#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;


#ifdef HAVE_LIBBOOST_THREAD
#include <boost/thread.hpp>
#endif

#define SOT_CHECK_TIME 
#define SOT_TIME_FILENAME "/tmp/dt.dat"

/* -------------------------------------------------------------------------- */
/* --- CLASS ---------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
enum robotType
  {
    hrp2_10_small,           //! Control new HRP-2 10 with new HRP-2 10 small model
    hrp2_10_small_old,        //! Control new HRP-2 10 with old HRP-2 10 small model
    hrp2_14_small            //! Control HRP-2 14 with its small model
  };

class StackOfTasks
:public plugin 
  ,public sotEntity
{
    
public:  /* --- GENERIC PLUGIN IMPLEMENTATION --- */

  /** @name GenericPlugin
   * Generic plugin implementation: implements the method inherited from
   * the plugin class. 
   */
  // @{ 
  StackOfTasks( istringstream &strm, int lnbDofs, robotType aRobotToControl );
  virtual ~StackOfTasks( void );
  bool setup  ( sotRobotState* rs, sotMotorCommand* mc );  
  void control( sotRobotState* rs, sotMotorCommand* mc );
  bool cleanup( sotRobotState* rs, sotMotorCommand* mc );
  // @}


public:  /* --- SPECIFIC PLUGIN IMPLEMENTATION --- */

  enum sotPluginState
    {
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

  sotPluginState pluginState;

  /* ! Specify the reference state. */
  enum sotReferenceState
  {
    REFSTATE_RS = 0x00, //! The default.
    REFSTATE_MC = 0x01  //! The conservative one.
  };

  void m_script( istringstream &strm );
  void m_init( istringstream &strm );
  void m_setForces( istringstream &strm );

  /* @name ProperFinishing 
     This method makes sure that the seq player internal
     state comes back to the current situation. It usually
     assumes that the WPG has finish its work.
   */
  // @{
  void m_hold(istringstream &strm);
  void m_waitForFinished( istringstream &strm );
  // @}
  
  void m_test(istringstream &strm);
  //void m_distantShell(istringstream &strm);

  void sotInit( void );
  void sotControlLoop( sotRobotState* rs, sotMotorCommand* mc );
  void sotControlFinishing( sotRobotState* rs, sotMotorCommand* mc );


 protected: /* --- MEMBERS --- */
  sotPluginLoader pluginLoader;
  /*! Reference to the sequence player plugin. */
  sotSequencePlayer_var seqpluginPTR;

 public:
#ifdef WALK_PG_JRL
  walkpluginJRL_ptr walkGenpluginPTR;
#endif
  bool suspend;

# ifdef SOT_CHECK_TIME 
  static const int TIME_ARRAY_SIZE = 100000;
  double timeArray[TIME_ARRAY_SIZE];
  int stateArray[TIME_ARRAY_SIZE];
  int timeIndex; 
# endif  // #ifdef SOT_CHECK_TIME 

  static const double TIMESTEP_DEFAULT;
  double timestep;

 protected: 
  /*! Used to handle different robots */
  string m_RobotName;
  unsigned int m_nbDofs;
  robotType m_robotToControl;

  void updateHRP2SmallFromMC(sotMotorCommand *mc, 
			     maal::boost::Vector &VectorCommand,
			     unsigned int ref=0);
  void updateHRP2SmallMC(sotMotorCommand *mc, 
			 maal::boost::Vector &VectorCommand,
			 unsigned int ref=0);
  void updateHRP2Smalltorque(sotMotorCommand *rs,
			     maal::boost::Vector &Torque);

  void updateHRP210SmallOldFromMC(sotMotorCommand *mc, 
				  maal::boost::Vector &VectorCommand,
				  unsigned int ref=0);
  void updateHRP210SmallOldMC(sotMotorCommand *mc, 
			      maal::boost::Vector &VectorCommand,
			      unsigned int ref =0);
  
  void updateHRP210SmallOldtorque(sotMotorCommand *rs,
				  maal::boost::Vector &Torque);
  
 public: /* --- SOT ENTITY --- */

  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

  void commandLine( const std::string& cmdLine,std::istringstream& cmdArgs,
		    std::ostream& os );

  void updateMC(sotMotorCommand *mc, maal::boost::Vector &VectorCommand);
  void updateFromMC(sotMotorCommand *mc, 
		    maal::boost::Vector &VectorCommand,
		    unsigned int ref=0);
  void updateTorque(sotRobotState *rs,
		    maal::boost::Vector &torque);

  void play( void );
  void pause( void );

  //! Specific fields of the 
  unsigned int iter;
  maal::boost::Vector robotStatePrec; 
  bool robotStatePrecInit;
  bool reinitFromMc;

  maal::boost::Vector motorCommandPrec;
  bool motorCommandInit;

#ifdef HAVE_LIBBOOST_THREAD
  boost::try_mutex controlMutex;
#endif

  enum ForceSignalSource
  {
    FORCE_SIGNAL_RLEG,
    FORCE_SIGNAL_LLEG,
    FORCE_SIGNAL_RARM,
    FORCE_SIGNAL_LARM
  };
  bool withForceSignals[4];

  sotSignalPtr<ml::Vector,int> controlSIN;
  sotSignalPtr<ml::Vector,int> attitudeSIN;
  sotSignalPtr<sotMatrixHomogeneous,int> positionSIN;
  sotSignalPtr<ml::Vector,int> zmpSIN;

  sotSignal<ml::Vector,int> stateSOUT;
  sotSignal<ml::Vector,int>* forcesSOUT[4];
  sotSignal<sotMatrixRotation,int> attitudeSOUT;

  sotSignal<ml::Vector,int> pseudoTorqueSOUT;
  bool activatePseudoTorqueSignal;
  //sotSignal<ml::Vector,int> velocitySOUT;
  //bool activateVelocitySignal;
  sotSignal<ml::Vector,int> previousStateSOUT;
  sotSignal<ml::Vector,int> previousControlSOUT;
  /*! \brief The current state of the robot from the command viewpoint. */
  sotSignal<ml::Vector,int> motorcontrolSOUT;
  /*! \brief The ZMP reference send by the previous controller. */
  sotSignal<ml::Vector,int> ZMPPreviousControllerSOUT;
  bool activatePreviousControlSignal;

  /*! Peridic call that is called /before/ the MC computation
   * (ie for precomputation needed by the main graph, like 
   * event FSM-based computations). */
  sotPeriodicCall periodicCallBefore;

  /*! Peridic call that is called /after/ the MC computation (eg
   * for tracing). */
  sotPeriodicCall periodicCallAfter;

  /*! Member storing the reference state.  */
  unsigned int m_ReferenceState;
    
};




#endif /* #ifndef __STACK_OF_TASKS_OPENHRP_PLUGIN_H__ */
