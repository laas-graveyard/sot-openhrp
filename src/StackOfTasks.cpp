/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright JRL-Japan, 2010
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      StackOfTasks.cpp
 * Project:   SOT
 * Author:    Olivier Stasse, Nicolas Mansard
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/* -------------------------------------------------------------------------- */
/* --- INCLUDE -------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

//#define VP_DEBUG 
//#define VP_DEBUG_MODE 40
#define SOT_CHECK_TIME
#include "StackOfTasks.h"              /* Header of the below-defined class. */

#include <fstream>
#include <string>
#include <dlfcn.h> 
#include <sys/time.h>

#include <sot/sotDebug.h>

using namespace std;

plugin* create_plugin(istringstream &strm) 
{
  unsigned int lnbDofs = 46;
  robotType aRobotToControl=hrp2_14_small;
  string lRobotName;
  strm >> lRobotName;

  if (lRobotName == "HRP2JRL10SmallOld")
    {
      lnbDofs = 46;
      aRobotToControl = hrp2_10_small_old;
    }
  else if (lRobotName == "HRP2JRL10Small")
    {
      lnbDofs = 48;
      aRobotToControl = hrp2_10_small;
    }
  ofstream aof;
  aof.open("/tmp/starting.txt");
  aof << "Robot name: " << lRobotName <<endl;
  aof << "NbDofs:" << lnbDofs << endl;
  aof << "aRobotToControl:" << aRobotToControl << endl;
  aof.close();
  return new StackOfTasks(strm,lnbDofs,aRobotToControl);
}

/* -------------------------------------------------------------------------- */
/* --- CLASS ---------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

static bool admissibleState( StackOfTasks::sotPluginState state,char admis )
{
  return ( state&admis );
}

const std::string StackOfTasks::CLASS_NAME = "OpenHRP";

static std::string HRPname = "OpenHRP";
const double StackOfTasks::TIMESTEP_DEFAULT = 0.005;

/* --- GENERIC PLUGIN IMPLEMENTATION ---------------------------------------- */
StackOfTasks::
StackOfTasks( istringstream &strm, int lnbDofs, robotType aRobotToControl)
  :sotEntity(HRPname)//StackOfTasks::CLASS_NAME)
   ,pluginState(sotJUST_BUILT)

   ,pluginLoader()

   ,seqpluginPTR(NULL)
   ,suspend(true)
# ifdef SOT_CHECK_TIME 
   ,timeIndex(0)
#endif // #ifdef SOT_CHECK_TIME 
  ,timestep( TIMESTEP_DEFAULT )
  ,m_nbDofs(lnbDofs)
  ,m_robotToControl(aRobotToControl)
  ,iter(0)
  ,robotStatePrec( lnbDofs-6 )
  ,robotStatePrecInit(false)
  ,reinitFromMc(true)
   
  ,motorCommandPrec( lnbDofs-6 )
  ,motorCommandInit( false )
   
  ,controlSIN(NULL,"OpenHRPplugin("+HRPname+")::input(vector)::control")
  ,attitudeSIN(NULL,"OpenHRPplugin("+HRPname+")::input(vector3)::attitudeIN")
  ,positionSIN(NULL,"OpenHRPplugin("+HRPname+")::input(sotMatrixHomogeneous)::positionIN")
  ,zmpSIN(NULL,"OpenHRPplugin("+HRPname+")::input(vector3)::zmp")
  ,stateSOUT( "OpenHRPplugin("+HRPname+")::output(vector)::state" )
  ,attitudeSOUT( "OpenHRPplugin("+HRPname+")::output(sotMatrixRotation)::attitude" )
   
  ,pseudoTorqueSOUT( "OpenHRPplugin("+HRPname+")::output(vector)::ptorque" )
  ,activatePseudoTorqueSignal(false)
  ,previousStateSOUT("OpenHRPplug("+HRPname+")::output(vector)::previousState")
  ,previousControlSOUT( "OpenHRPplugin("+HRPname+")::output(vector)::previouscontrol" )
  ,motorcontrolSOUT("OpenHRPplugin("+HRPname+")::output(vector)::motorcontrol")
  ,ZMPPreviousControllerSOUT("OpenHRPplugin("+HRPname+")::output(vector)::zmppreviouscontroller") 
  ,activatePreviousControlSignal(false)   
{
  m_ReferenceState = REFSTATE_RS;

  if( sotDEBUG_ENABLE(1) ) { sotDebugTrace::openFile(); }
  sotDEBUGIN(5);

  assigned_time = 0.005;

  /* --- FORCES --- */
  for( int i=0;i<4;++i ){ withForceSignals[i] = false; }
  forcesSOUT[0] =
    new sotSignal<ml::Vector, int>("OpenHRPplugin("+HRPname+")::output(vector6)::forceRLEG");
  forcesSOUT[1] =
    new sotSignal<ml::Vector, int>("OpenHRPplugin("+HRPname+")::output(vector6)::forceLLEG");
  forcesSOUT[2] =
    new sotSignal<ml::Vector, int>("OpenHRPplugin("+HRPname+")::output(vector6)::forceRARM");
  forcesSOUT[3] =
    new sotSignal<ml::Vector, int>("OpenHRPplugin("+HRPname+")::output(vector6)::forceLARM");

  register_method(":init",(method)&StackOfTasks::m_init);
  register_method(":script",(method)&StackOfTasks::m_script);
  register_method(":setForces",(method)&StackOfTasks::m_setForces);
  register_method(":hold",(method)&StackOfTasks::m_hold);
  register_method(":test",(method)&StackOfTasks::m_test);
  register_method(":waitForFinished",(method)&StackOfTasks::m_waitForFinished);

  motorcontrolSOUT.setConstant(robotStatePrec);
  motorcontrolSOUT.setTime(iter);

  signalRegistration( controlSIN
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
  {
    void * dlib = dlopen( "libsot-0.so",RTLD_NOW|RTLD_GLOBAL);
    if( NULL==dlib ) 
      {
	sotDEBUG(5) << "Failure while loading: " <<dlerror() <<endl;
	SOT_THROW sotExceptionFactory( sotExceptionFactory::DYNAMIC_LOADING,
				       "Error while dlopen. ",dlerror() );
      }
    void * dlib2 = dlopen( "StackOfTasks.so",RTLD_NOW|RTLD_GLOBAL);
    if( NULL==dlib2 ) 
      {
	sotDEBUG(5) << "Failure while loading StackOfTasks.so: " <<dlerror() <<endl;
	SOT_THROW sotExceptionFactory( sotExceptionFactory::DYNAMIC_LOADING,
				       "Error while dlopen. ",dlerror() );
      }
  }

  std::cout << setprecision (9);

  sotDEBUGOUT(5);
}


 
StackOfTasks::
~StackOfTasks( void )
{
  sotDEBUGIN(5);
  if(! admissibleState( pluginState,sotCLEANED_UP )) ; // TRACE

# ifdef SOT_CHECK_TIME 
  ofstream of( "/tmp/nmansard/dt.dat" );
  for( int i=0;i<timeIndex;++i )
    {
      of<<i<<"\t"<<timeArray[i]<<"\t"<<stateArray[i]<<endl;
    }
# endif  // #ifdef SOT_CHECK_TIME 

  for( int i=0;i<4;++i ) { delete forcesSOUT[i]; }

  sotDEBUGOUT(5);
}
 

bool StackOfTasks::
setup( sotRobotState* rs, sotMotorCommand* mc )
{
  sotDEBUG(25) << "Iter = "<<iter 
	       <<"   state = "<< pluginState << endl;
# ifdef SOT_CHECK_TIME 
  struct timeval t0,t1;
  gettimeofday(&t0,NULL);
# endif // ifdef SOT_CHECK_TIME 

  sotDEBUGIN(5);
  if(! admissibleState( pluginState,sotINIT )) return false;
  
  ml::Vector mlrs(m_nbDofs);
  sotDEBUG(25) << endl; 
  //  for( int i=0;i<40;++i )
  //    {mlrs(i+6) = rs->angle[i]; }
  updateFromMC(rs,mlrs,6);
  stateSOUT.setConstant( mlrs );
  stateSOUT.setTime( iter );

  ml::Vector mlforces(6);
  for( int i=0;i<4;++i )
  {
    if( withForceSignals[i] == true )
    {
      for( int j=0;j<6;++j )
	mlforces(j) = rs->force[i][j];
      forcesSOUT[i]->setConstant( mlforces );
      forcesSOUT[i]->setTime( iter );
    }
  }

  sotDEBUG(25) << mlrs <<endl;

  pluginState = sotSETUP;
# ifdef SOT_CHECK_TIME 
  gettimeofday(&t1,NULL);
  double dt = ( (t1.tv_sec-t0.tv_sec) * 1000.
		+ (t1.tv_usec-t0.tv_usec+0.) / 1000. );
  if( timeIndex<TIME_ARRAY_SIZE ) 
    {
      timeArray[ timeIndex ] = dt;
      stateArray[ timeIndex++ ] = pluginState;
    }

# endif // ifdef SOT_CHECK_TIME 
  sotDEBUGOUT(5);

  return true;
}

void StackOfTasks::
control( sotRobotState* rs, sotMotorCommand* mc )
{  
  sotDEBUG(25) << "Iter = "<<iter 
	       <<"   state = "<< pluginState << endl;

# ifdef SOT_CHECK_TIME 
  struct timeval t0,t1;
  gettimeofday(&t0,NULL);
# endif // ifdef SOT_CHECK_TIME 

  switch( pluginState )
    {
    case sotSETUP: pluginState = sotLOOP; // no break
    case sotLOOP: 
      sotControlLoop( rs,mc );
      break;
    case sotFINISHING:
    case sotHOLD:
      sotControlFinishing( rs,mc );
      break;
    default:
      break;
    }

# ifdef SOT_CHECK_TIME 
  gettimeofday(&t1,NULL);
  double dt = ( (t1.tv_sec-t0.tv_sec) * 1000.
		+ (t1.tv_usec-t0.tv_usec+0.) / 1000. );
  if( timeIndex<TIME_ARRAY_SIZE ) 
    {
      timeArray[ timeIndex ] = dt;
      stateArray[ timeIndex++ ] = pluginState;
    }
  sotDEBUG(25) << "dt = " << dt << endl;
# endif // ifdef SOT_CHECK_TIME 

  return;
}

bool StackOfTasks::
cleanup( sotRobotState* rs, sotMotorCommand* mc )
{
  sotDEBUGIN(5);
# ifdef SOT_CHECK_TIME 
  struct timeval t0,t1;
  gettimeofday(&t0,NULL);
# endif // ifdef SOT_CHECK_TIME 

  sotDEBUG(25) << "Iter = "<<iter 
	       <<"   state = "<< pluginState << endl;
  
  if( admissibleState( pluginState,sotLOOP ) ) 
    {
      pluginState = sotHOLD; 
      sotControlFinishing( rs,mc ); 
      return false; 
    }
  if( admissibleState( pluginState,sotFINISHING )) sotControlFinishing(rs,mc);
  if(! (admissibleState( pluginState,sotFINISHED )
	||admissibleState( pluginState,sotCLEANED_UP )) ) return false;
  pluginLoader.unloadAllPlugins();
  pluginState = sotCLEANED_UP;

# ifdef SOT_CHECK_TIME 
  gettimeofday(&t1,NULL);
  double dt = ( (t1.tv_sec-t0.tv_sec) * 1000.
		+ (t1.tv_usec-t0.tv_usec+0.) / 1000. );
  if( timeIndex<TIME_ARRAY_SIZE ) 
    {
      timeArray[ timeIndex ] = dt;
      stateArray[ timeIndex++ ] = pluginState;
    }

# endif // ifdef SOT_CHECK_TIME 

  sotDEBUGOUT(5);
  return true;
}



/* --- SPECIFIC PLUGIN IMPLEMENTATION --------------------------------------- */
#include <sot/sotExceptionTask.h>

void StackOfTasks::
m_script( istringstream &strm )
{
  sotDEBUGIN(5);

  string cmdLine;
  strm >>skipws>> cmdLine ;
  sotDEBUG(1) << "Cmd <" <<cmdLine<<">"<<endl;

  try {
    sotShell.cmd( cmdLine,strm, std::cout );
  } catch( const sotExceptionAbstract& e ) { std::cout << "!! " << e <<endl; }

  sotDEBUGOUT(5);
}

void StackOfTasks::
m_init(istringstream &strm)
{
  sotDEBUGIN(5);
  sotInit();  if(0){
    void * dlib = dlopen( "libboost_thread-mt.so",
			  RTLD_NOW|RTLD_GLOBAL);
    if( NULL==dlib ) 
      {
	sotDEBUG(5) << "Failure while loading: " <<dlerror() <<endl;
	SOT_THROW sotExceptionFactory( sotExceptionFactory::DYNAMIC_LOADING,
				       "Error while dlopen. ",dlerror() );
      }
  }

  sotDEBUGOUT(5);
}

void StackOfTasks::
m_setForces(istringstream &strm)
{
  sotDEBUGIN(5);

  int i = -1;
  strm >> i;
  bool onOff = false;
  strm >> onOff;

  if((i >= 0) && (i < 4))
    withForceSignals[i] = onOff;

  sotDEBUGOUT(5);
}

void StackOfTasks::
m_hold( istringstream &strm )
{
  pluginState = sotHOLD;
}

void StackOfTasks::
m_test( istringstream &strm )
{
  sotDEBUGIN(5);
  sotDEBUGOUT(5);
}

void StackOfTasks::
m_waitForFinished( istringstream &strm )
{
  sotDEBUGIN(15);
  do
    {
      usleep( 1000*100 );
    } while  (! admissibleState( pluginState,sotFINISHED )) ;

  sotDEBUGOUT(15);
  return;
}

/* --- MEMBERS -------------------------------------------------------------- */
/* --- MEMBERS -------------------------------------------------------------- */
/* --- MEMBERS -------------------------------------------------------------- */

void StackOfTasks::
sotInit( void )
{
  sotDEBUGIN(5);

  sotShell.referencePluginLoader(&pluginLoader);

  seqpluginPTR = sotSequencePlayer::_narrow(manager->find("seq",""));
  sotDEBUG(25) << "Seqplugin: " << seqpluginPTR <<endl;

#ifdef WALK_PG_JRL
  walkGenpluginPTR = walkpluginJRL::_narrow(manager->find("walk",""));
#endif

  iter = 0;
  pluginState = sotINIT;

  sotDEBUGOUT(5);
}


void StackOfTasks::updateMC(sotMotorCommand *mc,
			    maal::boost::Vector &VectorCommand)
{
  switch(m_robotToControl)
    {
    case hrp2_10_small:
    case hrp2_14_small:
      updateHRP2SmallMC(mc,VectorCommand,0);
      break;
    case hrp2_10_small_old:
      updateHRP210SmallOldMC(mc,VectorCommand,0);
      break;
    }
  
}

void StackOfTasks::updateFromMC(sotMotorCommand *mc,
				maal::boost::Vector &VectorCommand,
				unsigned int ref)
{
  switch(m_robotToControl)
    {
    case hrp2_10_small:
    case hrp2_14_small:
      updateHRP2SmallFromMC(mc,VectorCommand);
      break;
    case hrp2_10_small_old:
      updateHRP210SmallOldFromMC(mc,VectorCommand);
      break;
    default:
      sotDEBUG(1) << "No robot specified" << endl;
      break;
    }
}

void StackOfTasks::updateTorque(sotRobotState *rs,
				maal::boost::Vector &torque)
{
  switch(m_robotToControl)
    {
    case hrp2_10_small:
    case hrp2_14_small:
      updateHRP2Smalltorque(rs,torque);
      break;
    case hrp2_10_small_old:
      updateHRP210SmallOldtorque(rs,torque);
      break;
    }
}

void StackOfTasks::
sotControlLoop( sotRobotState* rs, sotMotorCommand* mc )
{
  sotDEBUGIN(5) << std::endl;

  if( !admissibleState( pluginState,sotLOOP )) return;
  iter ++; 
  const unsigned int iterCurrent = iter;
  sotDEBUG(15) << "--- Iter = "<<iter 
	       <<"-------------------------------------------------"<<endl;

  try {
#ifdef HAVE_LIBBOOST_THREAD
    boost::try_mutex::scoped_try_lock lock(controlMutex);
#endif
      
    /* --- RS --- */
    ml::Vector mlrs(m_nbDofs);
    sotDEBUG(25) << endl;
    for( int i=0;i<6;++i )mlrs(i)=0.;

    if (m_ReferenceState==REFSTATE_RS)
      {
	for(unsigned int i=0;i<m_nbDofs-6;++i )
	  {mlrs(i+6) = rs->angle[i]; }
      }
    else if (m_ReferenceState==REFSTATE_MC)
      {
	for(unsigned int i=0;i<m_nbDofs-6;++i )
	  mlrs(i+6) = robotStatePrec(i);
      }

    stateSOUT.setConstant( mlrs );
    stateSOUT.setTime( iter );

    ml::Vector ZMPPrevContr(3);
    for(unsigned int i=0;i<3;i++)
      ZMPPrevContr(i) = mc->zmp[i];
  
    ZMPPreviousControllerSOUT.setConstant(ZMPPrevContr);
    ZMPPreviousControllerSOUT.setTime(iter);

    sotDEBUG(25) << mlrs <<endl;
    if(! robotStatePrecInit )
      {
	sotDEBUG(25) << "Reinit RS." <<endl;
	if (reinitFromMc)
	  {
	    updateFromMC(mc,robotStatePrec);
	  }
	else
	  {
	    for(unsigned int i=0;i<m_nbDofs;++i )
	      robotStatePrec(i) = mlrs(i+6); 
	  }

      
	robotStatePrecInit = true;
      }
  

    /* --- FORCES --- */
    ml::Vector mlforces(6);
    for( int i=0;i<4;++i )
      {
	if( withForceSignals[i] == true )
	  {
	    for( int j=0;j<6;++j )
	      mlforces(j) = rs->force[i][j];
	    forcesSOUT[i]->setConstant( mlforces );
	    forcesSOUT[i]->setTime( iter );
	  }
      }
    sotDEBUG(15) << "force = "<< rs->force.length() << std::endl;

    /* --- KF --- */
    sotMatrixRotation mlkf;  
    if( rs->attitude.length() )
      {
	sotDEBUG( 25 ) << "Get KF " << rs->attitude[0].length() << endl;
	for( unsigned int i=0;i<rs->attitude[0].length();++i ) 
	  sotDEBUG( 25 ) << "Value  " << i << " " << rs->attitude[0][i] <<endl;

	/* --- HACK: in simulation, attitude is a rotation matrix, while
	 * on the real robot, attitude is a quaternion --- */

	if(rs->attitude[0].length() == 4) { // we have a quaternion: convert it to a matrix
	  double qw = rs->attitude[0][0];
	  double qx = rs->attitude[0][1];
	  double qy = rs->attitude[0][2];
	  double qz = rs->attitude[0][3];
	  double qxqy = qx * qy;
	  double qxqw = qx * qw;
	  double qxqz = qx * qz;
	  double qyqz = qy * qz;
	  double qyqw = qy * qw;
	  double qzqw = qz * qw;
	  double qx2 = qx * qx;
	  double qy2 = qy * qy;
	  double qz2 = qz * qz;

	  mlkf(0,0) = 1 - 2*qy2 - 2*qz2;
	  mlkf(0,1) = 2*qxqy - 2*qzqw;
	  mlkf(0,2) = 2*qxqz + 2*qyqw;

	  mlkf(1,0) = 2*qxqy + 2*qzqw;
	  mlkf(1,1) = 1 - 2*qx2 - 2*qz2;
	  mlkf(1,2) = 2*qyqz - 2*qxqw;

	  mlkf(2,0) = 2*qxqz - 2*qyqw;
	  mlkf(2,1) = 2*qyqz + 2*qxqw;
	  mlkf(2,2) = 1 - 2*qx2 - 2*qy2;
	}
	else      /* --- END OF HACK --- */
	  if(rs->attitude[0].length() == 9) 
	    { // we have a quaternion: convert it to a matrix
	      for( unsigned int i=0;i<3;++i ) 
		for( unsigned int j=0;j<3;++j ) 
		  mlkf(i,j)=rs->attitude[0][i*3+j];
	    }
      }
    else
      mlkf.setIdentity();

    if( isnan(mlkf(1,1)) ) mlkf.setIdentity();

    sotDEBUG(25) << "Before setting attitudeSOUT: " <<mlkf << std::endl;
    attitudeSOUT.setConstant( mlkf );
    sotDEBUG(25) << mlkf <<endl;
    sotDEBUG(25) << "rs->torque.length():" << rs->torque.length() <<endl;
    /* --- OTHER SIG --- */
    if( activatePseudoTorqueSignal&&rs->torque.length() )
      {
	//	ml::Vector torque(SIZE);
	//	for( unsigned int i=0;i<SIZE;++i ) torque(i)=rs->torque[i];
	ml::Vector torque(m_nbDofs);

	updateTorque(rs,torque);

	pseudoTorqueSOUT.setConstant( torque );
	pseudoTorqueSOUT.setTime( iter );
	
      }
    if( activatePreviousControlSignal&&(robotStatePrec.size()>0) )
      {
	const double dtinv= 1./timestep;
	ml::Vector previousControl(robotStatePrec.size());
	ml::Vector previousmc(robotStatePrec.size());
	robotStatePrec.opposite(previousmc);

	updateFromMC(mc,previousmc);
	for(unsigned int i=0;i<m_nbDofs-6;++i ) 
	  {
	    previousControl(i) += previousmc(i);
	    previousControl(i) *= dtinv;
	  }
	previousControlSOUT = previousControl;
      }

    /* --- TIME OVERLAPS --- */
    sotDEBUGF(15,"In case of overshoot of the timeline, "
	     "copy the previous command in mc (init=%d).",motorCommandInit);
    if( motorCommandInit ) robotStatePrec += (motorCommandPrec*0.005);
    updateMC(mc,robotStatePrec);
    
    /* --- PERIODIC CALL --- */
    periodicCallBefore.run( iter );
    
    /* --- MC --- */
    try
      {
	if(! suspend)
	  {
	    sotDEBUG(25) << robotStatePrec <<endl;
	    const ml::Vector &mlmc = controlSIN(iter);
	    bool mcIsNan = false;
	    for(unsigned int i=0;i<m_nbDofs-6;++i ) if( isnan(mlmc(i)) ) 
	      { mcIsNan=true; break; }
	    if( mcIsNan ) 
	      { controlSIN.unplug(); }
	    else 
	      {
		sotDEBUG(25) << mlmc <<endl;
		
		if( (iter==iterCurrent)&&motorCommandInit )
		  { /* No overlaps with next iteration. */
		    robotStatePrec -= (motorCommandPrec*0.005);
		  }
		motorCommandPrec = mlmc; motorCommandInit = true;
		if( iter==iterCurrent )
		  {
		    robotStatePrec += (motorCommandPrec*0.005);
		    //for( int i=0;i<40;++i ) 
		    //		      { mc->angle[i] = robotStatePrec(i); }
		    updateMC(mc,robotStatePrec);
		    sotDEBUG(15) << "MotorControl :" 
				 << robotStatePrec <<endl;

		    /* Update motor control output signal. */
		    /* Nico: strange to have that here. Not before? Check Oliv. */
		    motorcontrolSOUT.setConstant(robotStatePrec);
		    motorcontrolSOUT.setTime(iter);
    		  }
		else { sotDEBUG(1) << "Time overlaps : "
				   << iter <<"!="<<iterCurrent<<std::endl; }
	      }
	  } else { motorCommandInit = false; }
      }
    catch( const sotExceptionAbstract& e ) {sotDEBUG(1) << e;}
    catch( ... ) { sotDEBUG(1) << "Unknown catched." <<endl; }
  
    /* --- WAIST POSITION --- */
    try {
      if (!suspend)
	{
	  const sotMatrixHomogeneous & positionmc = positionSIN(iter);
	  sotMatrixRotation aRot; positionmc.extract(aRot);
	  if (iter==iterCurrent)
	    {
#ifdef OPENHRP_VERSION_3
	      for(int i=0;i<3;i++)
		mc->basePos[i] = positionmc(i,3);

	      for(unsigned int i=0;i<3;i++)
		for(unsigned int j=0;j<3;j++)
		  mc->baseAtt[i*3+j] = aRot(i,j);

#endif
	    }
	}
    }
    catch(...)
      {
	/*
	cout << "Catched a problem with mc->basePos "
	     << mc->basePos.length() << endl;
	*/
      }

    /* --- WAIST ATTITUDE --- */
    try {
      if(! suspend )
	{
#ifdef OPENHRP_VERSION_2
	  const ml::Vector & attitudemc = attitudeSIN(iter);
#endif  
	  if( iter==iterCurrent )
	    {
#ifdef OPENHRP_VERSION_2
	      for( int i=0;i<3;++i ) 
		mc->waistRpy[i] = attitudemc(i);
#endif

#if 0    
#ifdef OPENHRP_VERSION_3
	      sotMatrixRotation rot;
	      const double cr = cos(attitudemc(0)); // ROLL
	      const double sr = sin(attitudemc(0));
	      const double cp = cos(attitudemc(1)); // PITCH
	      const double sp = sin(attitudemc(1));
	      const double cy = cos(attitudemc(2)); // YAW
	      const double sy = sin(attitudemc(2));
	      
	      rot(0,0) = cy*cp;
	      rot(0,1) = cy*sp*sr-sy*cr;
	      rot(0,2) = cy*sp*cr+sy*sr;
	      
	      rot(1,0) = sy*cp;
	      rot(1,1) = sy*sp*sr+cy*cr;
	      rot(1,2) = sy*sp*cr-cy*sr;
	      
	      rot(2,0) = -sp;
	      rot(2,1) = cp*sr;
	      rot(2,2) = cp*cr;
	      
	      for(unsigned int i=0;i<3;i++)
		for(unsigned int j=0;j<3;j++)
		  mc->baseAtt[i*3+j] = rot(i,j);
#endif
#endif
	    }
	}
    } 
    catch (...) 
      { 
      }
    
    /* --- ZMP REF --- */
    try {
      if(! suspend ) 
	{
	  const ml::Vector & zmpmc = zmpSIN(iter); 
	  sotDEBUG(25) << "Copy zmpref:"<< zmpmc << endl;
	  if( iter==iterCurrent )
	    for( int i=0;i<3;++i ) mc->zmp[i] = zmpmc(i);
	}
    } catch (...) {}
 

    /* --- PERIODIC CALL --- */
    periodicCallAfter.run( iter );

    
  } 

#ifdef HAVE_LIBBOOST_THREAD
  catch( boost::lock_error le ) 
    {
      sotDEBUG(1) << "Overlaps catched." << std::endl;
      if( motorCommandInit&&robotStatePrecInit ) 
	robotStatePrec += (motorCommandPrec*0.005);
      //      for( int i=0;i<40;++i ) 
      //	{ mc->angle[i] = robotStatePrec(i); }
      updateMC(mc,robotStatePrec);
    }
#endif

  previousStateSOUT.setConstant(robotStatePrec);

 sotDEBUGOUT(5) << std::endl; 
}

void StackOfTasks::
sotControlFinishing(sotRobotState *rs, sotMotorCommand *mc)
{ 
  sotDEBUG(15) << "--- Iter = "<<iter 
	       <<"-------------------------------------------------"<<endl;
  if( admissibleState( pluginState,sotHOLD )) 
  {
    pluginState=sotLOOP;    
    sotControlLoop(rs,mc);

    if( seqpluginPTR )
      {
	//seqpluginPTR->goActual();

	//	RobotState_var ref_state;	
	// 	for( int i=0;i<40;++i ) 
	// 	  {
	// 	    ref_state->angle[i] = robotStatePrec(i);
	// 	  }
	seqpluginPTR->setReferenceState(*mc,0.005);                              // send seq the next ref_state              
	seqpluginPTR->goHalfSitting(5.0);
      }
    for( unsigned int i=0;i<DOF;i++)
      {
	//mc->angle[i]=rs->angle[i];
	 //mc->angle[i] = m_PrevMc[i];
      }
    pluginState=sotFINISHING;
    return ;
  } 

  if( admissibleState( pluginState,sotFINISHING )) 
  {
    if( seqpluginPTR->isEmpty() ) pluginState=sotFINISHED;
  } 

  return ;
}

void StackOfTasks::play( void )
{
  if( suspend )
    {
      suspend = false;
      //robotStatePrecInit=false;
    }
}


void StackOfTasks::pause( void )
{
  if(!suspend) 
    {
      suspend = true;
      //robotStatePrecInit=false;
    }
}


void StackOfTasks::
commandLine( const std::string& cmdLine,std::istringstream& cmdArgs,std::ostream& os )
{
  if( cmdLine =="help" )
  {
    os << "Plugin OpenHRP: " << std::endl
       << " - play/pause: \tStart/Stop writing the controlSIN"
       << " in the robot MC." << endl
       << " - reinit [from {MC|RS}]: \treinit the current pos value (default MC)" <<endl
       << " - withForces <i> \tCopy the force sensor data from sensor " 
       << "<i> in the corresponding SOUT." <<endl
       << " - whichForces: \tDisplay the activated force signals." <<endl
       << " - with{PseudoTorque|PreviousControl} [{true|false}]: "
       << "activate/inactivate the signal." << endl
//        << " - with{Velocity|PseudoTorque} [{true|false}]: "
//        << "activate/inactivate the signal." << endl
       << " - periodicCall{Before|After}:\tAccess to the periodic caller. "
       << "\'Before\' and \'After\' refeere to the MC computation." <<endl
       << " - timestep [<value>]: get/set the timestep (=0.005)." << std::endl;
    sotEntity::commandLine(cmdLine,cmdArgs,os);
  }
  else if( cmdLine == "print" ) 
    {
      os << getName() << " [mode=" << ((suspend)?"pause":"play") <<"]."<<endl;
    }
  else if( cmdLine == "pause" ) { pause(); }
  else if( cmdLine == "play" )  { play(); }
  else if( cmdLine == "reinit" )
    {
      std::string fromMc;
      cmdArgs>>ws>>fromMc>>ws;
      if( ("from"==fromMc)&&(cmdArgs.good()) )
	{
	  cmdArgs>>fromMc;
	  if( fromMc=="rs" ) { robotStatePrecInit=false; reinitFromMc=false; }
	  else if( fromMc=="mc" ) { robotStatePrecInit=false; reinitFromMc=true; }
	}
      else { robotStatePrecInit=false; reinitFromMc=true; }
    }
  else if( cmdLine == "withForces" )
  {
    int index;
    cmdArgs >> index;
    if((index >= 0) && (index < 4))
      cmdArgs >> withForceSignals[index];
  }
  else if( cmdLine == "whichForces" )
  {
    os << "Force signals: " << endl; 
    for( unsigned int i=0;i<4;++i )
      {
	os << "\t- Force " << i << ": ";
	if( withForceSignals[i] )
	  { os << "Active";	  } else { os << "Inactive"; } 
	os << endl;
      }
  }
//   else if( cmdLine == "withVelocity" )
//   {
//     cmdArgs >> std::ws; if( cmdArgs.good() )
//       {
// 	std::string val; cmdArgs>>val; 
// 	activateVelocitySignal = ( (val=="true")||(val=="1") );
//       } else {
// 	os << "withVelocity = " << activateVelocitySignal << std::endl;
//       }
//   }
  else if( cmdLine == "withPseudoTorque" )
  {
    cmdArgs >> std::ws; if( cmdArgs.good() )
      {
	std::string val; cmdArgs>>val; 
	activatePseudoTorqueSignal = ( (val=="true")||(val=="1") );
      } else {
	os << "withPseudoTorque = " << activatePseudoTorqueSignal << std::endl;
      }
  }
  else if( cmdLine == "withPreviousControl" )
  {
    cmdArgs >> std::ws; if( cmdArgs.good() )
      {
	std::string val; cmdArgs>>val; 
	activatePreviousControlSignal = ( (val=="true")||(val=="1") );
      } else {
	os << "withPreviousControl = " << activatePreviousControlSignal << std::endl;
      }
  }
  // Kept two tests for compatibility reason. 
  else if(( cmdLine == "periodicCall" )||( cmdLine == "periodicCallAfter" ))
    { 
      string cmd2; cmdArgs >> cmd2;
      periodicCallAfter .commandLine( cmd2,cmdArgs,os );
    }
  else if( cmdLine == "periodicCallBefore" )
    { 
      string cmd2; cmdArgs >> cmd2;
      periodicCallBefore .commandLine( cmd2,cmdArgs,os );
    }
  else if( cmdLine == "timestep" )
    { 
      cmdArgs >> std::ws;
      if( cmdArgs.good() ) { cmdArgs >> timestep; } 
      else { os << "timestep = " << timestep << std::endl; }
    }
  else if (cmdLine == "refstate" )
    {
      string refstate;
      cmdArgs >> std::ws;
      if (cmdArgs.good() ) 
	{ 
	  cmdArgs >> refstate;
	  if (refstate=="rs")
	    m_ReferenceState = REFSTATE_RS;
	  if (refstate=="mc")
	    m_ReferenceState = REFSTATE_MC;
	}
      else
	{
	  os << "reference state ";
	  if (m_ReferenceState == REFSTATE_RS)
	    os << "rs";
	  else if (m_ReferenceState == REFSTATE_MC)
	    os << "mc";
	  else os << " pb...You should stop everything...";
	  os << endl;
	}
    }

  else sotEntity::commandLine(cmdLine,cmdArgs,os);

}

void StackOfTasks::updateHRP2SmallMC(sotMotorCommand *mc,
				     maal::boost::Vector &VectorCommand,
				     unsigned int ref)
{

  sotDEBUGIN(5);
  sotDEBUG(5) << "Vector.size():" <<VectorCommand.size() 
	      << " mc->angle length:" << mc->angle.length() <<endl; 

  unsigned int lindex=ref;
  for(unsigned int i=0;i<m_nbDofs-6;i++)
    mc->angle[i] = VectorCommand(lindex++);

  for(unsigned int i=0;i<VectorCommand.size();i++)
    {
      sotDEBUG(5) << VectorCommand(i)*180.0/M_PI << " ";
    }
  sotDEBUG(5)<< endl;

  for(unsigned int i=0;i<mc->angle.length();i++)
    {
      sotDEBUG(5) << mc->angle[i]*180.0/M_PI << " ";
    }
  sotDEBUG(5) <<endl;

  sotDEBUGOUT(5); 
  
}

void StackOfTasks::updateHRP2SmallFromMC(sotMotorCommand *mc,
					 maal::boost::Vector &VectorCommand,
					 unsigned int ref)
{
  unsigned int lindex=ref;
  std::ofstream aof;
  sotDEBUGIN(5);
  sotDEBUG(5) << "Vector.size():" <<VectorCommand.size() << " " << ref << endl;
  //  if (VectorCommand.size()!=40+ref)
  //    VectorCommand.resize(40+ref);

  sotDEBUG(5) << "Vector.size():" <<VectorCommand.size() 
	      << " mc->angle length:" << mc->angle.length() <<endl; 

  for(unsigned int i=0;i<m_nbDofs-6;i++)
    VectorCommand(lindex++) = mc->angle[i];

  for(unsigned int i=0;i<VectorCommand.size();i++)
    {
      sotDEBUG(5) <<aof << VectorCommand(i)*180.0/M_PI << " ";
    }
  sotDEBUG(5) << endl;

  for(unsigned int i=0;i<mc->angle.length();i++)
    {
      sotDEBUG(5) << mc->angle[i]*180.0/M_PI << " ";
    }
  sotDEBUG(5)<< endl;


  sotDEBUGOUT(5);
}

void StackOfTasks::updateHRP2Smalltorque(sotMotorCommand *rs,
					 maal::boost::Vector &Torque)
{
  Torque.resize(m_nbDofs);
  for(unsigned int i=0;i<m_nbDofs-6;i++)
    Torque(i) = rs->torque[i];
}

 
void StackOfTasks::updateHRP210SmallOldMC(sotMotorCommand *mc,
					  maal::boost::Vector &VectorCommand,
					  unsigned int ref)
{

  sotDEBUGIN(5);
  sotDEBUG(5) << "Vector.size():" <<VectorCommand.size() 
	      << " mc->angle length:" << mc->angle.length() <<endl; 

  unsigned int lindex=ref;
  for(unsigned int i=0;i<21;i++)
    mc->angle[i] = VectorCommand(lindex++);
  mc->angle[21] = 0.0;
  for(unsigned int i=22;i<29;i++)
    mc->angle[i] = VectorCommand(lindex++);
  mc->angle[29] = 0.0; 
  for(unsigned int i=30;i<42;i++)
    mc->angle[i] = VectorCommand(lindex++);

  for(unsigned int i=0;i<VectorCommand.size();i++)
    {
      sotDEBUG(5) << VectorCommand(i)*180.0/M_PI << " ";
    }
  sotDEBUG(5)<< endl;

  for(unsigned int i=0;i<mc->angle.length();i++)
    {
      sotDEBUG(5) << mc->angle[i]*180.0/M_PI << " ";
    }
  sotDEBUG(5) <<endl;

  sotDEBUGOUT(5); 
  
}

void StackOfTasks::updateHRP210SmallOldFromMC(sotMotorCommand *mc,
					      maal::boost::Vector &VectorCommand,
					      unsigned int ref)
{
  unsigned int lindex=ref;
  std::ofstream aof;
  sotDEBUGIN(5);
  sotDEBUG(5) << "Vector.size():" <<VectorCommand.size()  << endl;
  //  if (VectorCommand.size()!=40+ref)
  //    VectorCommand.resize(40+ref);

  sotDEBUG(5) << "Vector.size():" <<VectorCommand.size() 
	      << " mc->angle length:" << mc->angle.length() <<endl; 

  for(unsigned int i=0;i<21;i++)
    VectorCommand(lindex++) = mc->angle[i];
  for(unsigned int i=22;i<29;i++)
    VectorCommand(lindex++) = mc->angle[i];
  for(unsigned int i=30;i<42;i++)
    VectorCommand(lindex++) = mc->angle[i];

  for(unsigned int i=0;i<VectorCommand.size();i++)
    {
      sotDEBUG(5) <<aof << VectorCommand(i)*180.0/M_PI << " ";
    }
  sotDEBUG(5) << endl;

  for(unsigned int i=0;i<mc->angle.length();i++)
    {
      sotDEBUG(5) << mc->angle[i]*180.0/M_PI << " ";
    }
  sotDEBUG(5)<< endl;


  sotDEBUGOUT(5);
}

void StackOfTasks::updateHRP210SmallOldtorque(sotMotorCommand *rs,
					      maal::boost::Vector &Torque)
{
  unsigned int lindex = 0;
  Torque.resize(m_nbDofs);
  for(unsigned int i=0;i<21;i++)
    Torque(lindex++) = rs->angle[i];
  for(unsigned int i=22;i<29;i++)
    Torque(lindex++) = rs->angle[i];
  for(unsigned int i=30;i<42;i++)
    Torque(lindex++) = rs->angle[i];
}
