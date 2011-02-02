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
#include <sot-core/debug.h>
#include <sot-core/exception-factory.h>

#include "stack-of-tasks.hh"

#include "Plugin_impl.h"

using dynamicgraph::sot::openhrp::StackOfTasks;
const double StackOfTasks::TIMESTEP_DEFAULT = 0.005;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(StackOfTasks, "Robot");

StackOfTasks::StackOfTasks(const std::string& inName)
  :Entity(inName),
   iter_(0),
   robotStatePrec_((unsigned int)0),
   robotStatePrecInit_(false),
   reinitFromMc_(true),
   motorCommandPrec_((unsigned int)0),
   motorCommandInit_( false ),
   pluginState_(sotJUST_BUILT),
   suspend_(true),
# ifdef SOT_CHECK_TIME 
   timeIndex_(0),
#endif // #ifdef SOT_CHECK_TIME 
   timestep_( TIMESTEP_DEFAULT ),
   controlSIN(NULL,"OpenHRPplugin("+inName+")::input(vector)::control"),
   attitudeSIN(NULL,"OpenHRPplugin("+inName+")::input(vector3)::attitudeIN"),
   positionSIN(NULL,"OpenHRPplugin("+inName+")::input(sotMatrixHomogeneous)::positionIN"),
   zmpSIN(NULL,"OpenHRPplugin("+inName+")::input(vector3)::zmp"),
   stateSOUT( "OpenHRPplugin("+inName+")::output(vector)::state" ),
   attitudeSOUT( "OpenHRPplugin("+inName+")::output(sotMatrixRotation)::attitude" ),
   
   pseudoTorqueSOUT( "OpenHRPplugin("+inName+")::output(vector)::ptorque" ),
   activatePseudoTorqueSignal(false),
   previousStateSOUT("OpenHRPplug("+inName+")::output(vector)::previousState"),
   previousControlSOUT( "OpenHRPplugin("+inName+")::output(vector)::previouscontrol" ),
   motorcontrolSOUT("OpenHRPplugin("+inName+")::output(vector)::motorcontrol"),
   ZMPPreviousControllerSOUT("OpenHRPplugin("+inName+")::output(vector)::zmppreviouscontroller") ,
   activatePreviousControlSignal(false)
{
  referenceState_ = REFSTATE_RS;

#ifdef VP_DEBUG
  { DebugTrace::openFile(); }
#endif
  sotDEBUGIN(5);

  /* --- FORCES --- */
  for( int i=0;i<4;++i ){ withForceSignals[i] = false; }
  forcesSOUT[0] =
    new Signal<ml::Vector, int>("OpenHRPplugin("+inName+")::output(vector6)::forceRLEG");
  forcesSOUT[1] =
    new Signal<ml::Vector, int>("OpenHRPplugin("+inName+")::output(vector6)::forceLLEG");
  forcesSOUT[2] =
    new Signal<ml::Vector, int>("OpenHRPplugin("+inName+")::output(vector6)::forceRARM");
  forcesSOUT[3] =
    new Signal<ml::Vector, int>("OpenHRPplugin("+inName+")::output(vector6)::forceLARM");

  motorcontrolSOUT.setConstant(robotStatePrec_);
  motorcontrolSOUT.setTime(iter_);

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
  }

  std::cout << setprecision (9);

  sotDEBUGOUT(5);
}

bool StackOfTasks::setup( RobotState* rs, RobotState* )
{
  sotDEBUG(25) << "Iter = "<<iter_ 
	       <<"   state = "<< pluginState_ << endl;
# ifdef SOT_CHECK_TIME 
  struct timeval t0,t1;
  gettimeofday(&t0,NULL);
# endif // ifdef SOT_CHECK_TIME 

  sotDEBUGIN(5);
  if(pluginState_ != sotINIT) return false;
  
  ml::Vector mlrs(numberDofs_);
  sotDEBUG(25) << endl; 
  //  for( int i=0;i<40;++i )
  //    {mlrs(i+6) = rs->angle[i]; }
  updateFromMC(rs,mlrs,6);
  stateSOUT.setConstant( mlrs );
  stateSOUT.setTime( iter_ );

  ml::Vector mlforces(6);
  for( int i=0;i<4;++i )
  {
    if( withForceSignals[i] == true )
    {
      for( int j=0;j<6;++j )
	mlforces(j) = rs->force[i][j];
      forcesSOUT[i]->setConstant( mlforces );
      forcesSOUT[i]->setTime( iter_ );
    }
  }

  sotDEBUG(25) << mlrs <<endl;

  pluginState_ = sotSETUP;
# ifdef SOT_CHECK_TIME 
  gettimeofday(&t1,NULL);
  double dt = ( (t1.tv_sec-t0.tv_sec) * 1000.
		+ (t1.tv_usec-t0.tv_usec+0.) / 1000. );
  if( timeIndex_<TIME_ARRAY_SIZE ) 
    {
      timeArray[ timeIndex_ ] = dt;
      stateArray[ timeIndex_++ ] = pluginState_;
    }

# endif // ifdef SOT_CHECK_TIME 
  sotDEBUGOUT(5);

  return true;
}

void StackOfTasks::
control( RobotState* rs, RobotState* mc )
{  
  sotDEBUG(25) << "Iter = "<<iter_ 
	       <<"   state = "<< pluginState_ << endl;

# ifdef SOT_CHECK_TIME 
  struct timeval t0,t1;
  gettimeofday(&t0,NULL);
# endif // ifdef SOT_CHECK_TIME 

  switch( pluginState_ )
    {
    case sotSETUP: pluginState_ = sotLOOP; // no break
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
  if( timeIndex_<TIME_ARRAY_SIZE ) 
    {
      timeArray[ timeIndex_ ] = dt;
      stateArray[ timeIndex_++ ] = pluginState_;
    }
  sotDEBUG(25) << "dt = " << dt << endl;
# endif // ifdef SOT_CHECK_TIME 

  return;
}

bool StackOfTasks::
cleanup( RobotState*, RobotState* )
{
  return true;
}

StackOfTasks::~StackOfTasks()
{
  sotDEBUGIN(5);
  if(pluginState_ != sotCLEANED_UP) {}; // TRACE

# ifdef SOT_CHECK_TIME 
  ofstream of( "/tmp/nmansard/dt.dat" );
  for( int i=0;i<timeIndex_;++i )
    {
      of<<i<<"\t"<<timeArray[i]<<"\t"<<stateArray[i]<<endl;
    }
# endif  // #ifdef SOT_CHECK_TIME 

  for( int i=0;i<4;++i ) { delete forcesSOUT[i]; }

  sotDEBUGOUT(5);
}
 
void StackOfTasks::setNumberDofs(int numberDofs)
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
  motorCommandPrec_ = maal::boost::Vector((unsigned int)numberDofs-6);
}

void StackOfTasks::setRobotType(robotType whichRobot)
{
  whichRobot_ = whichRobot;
}

void StackOfTasks::sotInit()
{
  iter_ = 0;
  pluginState_ = sotINIT;

  sotDEBUGOUT(5);
}

void StackOfTasks::
sotControlLoop( RobotState* rs, RobotState* mc )
{
  sotDEBUGIN(5) << std::endl;

  if(pluginState_ != sotLOOP) return;
  iter_ ++; 
  const unsigned int iterCurrent = iter_;
  sotDEBUG(15) << "--- Iter = "<<iter_ 
	       <<"-------------------------------------------------"<<endl;

  try {
#ifdef HAVE_LIBBOOST_THREAD
    boost::try_mutex::scoped_try_lock lock(controlMutex_);
#endif
      
    /* --- RS --- */
    ml::Vector mlrs(numberDofs_);
    sotDEBUG(25) << endl;
    for( int i=0;i<6;++i )mlrs(i)=0.;

    if (referenceState_==REFSTATE_RS)
      {
	for(unsigned int i=0;i<numberDofs_-6;++i )
	  {mlrs(i+6) = rs->angle[i]; }
      }
    else if (referenceState_==REFSTATE_MC)
      {
	for(unsigned int i=0;i<numberDofs_-6;++i )
	  mlrs(i+6) = robotStatePrec_(i);
      }

    stateSOUT.setConstant( mlrs );
    stateSOUT.setTime( iter_ );

    ml::Vector ZMPPrevContr(3);
    for(unsigned int i=0;i<3;i++)
      ZMPPrevContr(i) = mc->zmp[i];
  
    ZMPPreviousControllerSOUT.setConstant(ZMPPrevContr);
    ZMPPreviousControllerSOUT.setTime(iter_);

    sotDEBUG(25) << mlrs <<endl;
    if(! robotStatePrecInit_ )
      {
	sotDEBUG(25) << "Reinit RS." <<endl;
	if (reinitFromMc_)
	  {
	    updateFromMC(mc,robotStatePrec_);
	  }
	else
	  {
	    for(unsigned int i=0;i<numberDofs_;++i )
	      robotStatePrec_(i) = mlrs(i+6); 
	  }

      
	robotStatePrecInit_ = true;
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
	    forcesSOUT[i]->setTime( iter_ );
	  }
      }
    sotDEBUG(15) << "force = "<< rs->force.length() << std::endl;

    /* --- KF --- */
    ::sot::MatrixRotation mlkf;  
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
	ml::Vector torque(numberDofs_);

	updateTorque(rs,torque);

	pseudoTorqueSOUT.setConstant( torque );
	pseudoTorqueSOUT.setTime( iter_ );
	
      }
    if( activatePreviousControlSignal&&(robotStatePrec_.size()>0) )
      {
	const double dtinv= 1./timestep_;
	ml::Vector previousControl(robotStatePrec_.size());
	ml::Vector previousmc(robotStatePrec_.size());
	robotStatePrec_.opposite(previousmc);

	updateFromMC(mc,previousmc);
	for(unsigned int i=0;i<numberDofs_-6;++i ) 
	  {
	    previousControl(i) += previousmc(i);
	    previousControl(i) *= dtinv;
	  }
	previousControlSOUT = previousControl;
      }

    /* --- TIME OVERLAPS --- */
    ::sot::sotDEBUGF(15,"In case of overshoot of the timeline, "
		     "copy the previous command in mc (init=%d).",
		     motorCommandInit_);
    if( motorCommandInit_ ) robotStatePrec_ += (motorCommandPrec_*0.005);
    updateMC(mc,robotStatePrec_);
    
    /* --- MC --- */
    try
      {
	if(! suspend_)
	  {
	    sotDEBUG(25) << robotStatePrec_ <<endl;
	    // trigger evaluation of the whole graph.
	    const ml::Vector &control = controlSIN(iter_);
	    ml::Vector mlmc;
	    if (control.size() == numberDofs_) {
	      // Free flyer is included in control vector
	      mlmc = control.extract(6, numberDofs_ - 6);
	    } else {
	      mlmc = control;
	    }
	    bool mcIsNan = false;
	    for(unsigned int i=0;i<numberDofs_-6;++i ) if( isnan(mlmc(i)) ) 
	      { mcIsNan=true; break; }
	    if( mcIsNan ) 
	      { controlSIN.unplug(); }
	    else 
	      {
		sotDEBUG(25) << mlmc <<endl;
		
		if( (iter_==iterCurrent)&&motorCommandInit_ )
		  { /* No overlaps with next iteration. */
		    robotStatePrec_ -= (motorCommandPrec_*0.005);
		  }
		motorCommandPrec_ = mlmc; motorCommandInit_ = true;
		if( iter_==iterCurrent )
		  {
		    robotStatePrec_ += (motorCommandPrec_*0.005);
		    //for( int i=0;i<40;++i ) 
		    //		      { mc->angle[i] = robotStatePrec_(i); }
		    updateMC(mc,robotStatePrec_);
		    sotDEBUG(15) << "MotorControl :" 
				 << robotStatePrec_ <<endl;

		    /* Update motor control output signal. */
		    /* Nico: strange to have that here. Not before? Check Oliv. */
		    motorcontrolSOUT.setConstant(robotStatePrec_);
		    motorcontrolSOUT.setTime(iter_);
    		  }
		else { sotDEBUG(1) << "Time overlaps : "
				   << iter_ <<"!="<<iterCurrent<<std::endl; }
	      }
	  } else { motorCommandInit_ = false; }
      }
    catch( const ExceptionAbstract& e ) {sotDEBUG(1) << e;}
    catch( ... ) { sotDEBUG(1) << "Unknown catched." <<endl; }
  
    /* --- WAIST POSITION --- */
    try {
      if (!suspend_)
	{
	  const ::sot::MatrixHomogeneous& positionmc = positionSIN(iter_);
	  ::sot::MatrixRotation aRot; positionmc.extract(aRot);
	  if (iter_==iterCurrent)
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
      if(! suspend_ )
	{
	  if( iter_==iterCurrent )
	    {
	    }
	}
    } 
    catch (...) 
      { 
      }
    
    /* --- ZMP REF --- */
    try {
      if(! suspend_ ) 
	{
	  const ml::Vector & zmpmc = zmpSIN(iter_); 
	  sotDEBUG(25) << "Copy zmpref:"<< zmpmc << endl;
	  if( iter_==iterCurrent )
	    for( int i=0;i<3;++i ) mc->zmp[i] = zmpmc(i);
	}
    } catch (...) {}
  } 

#ifdef HAVE_LIBBOOST_THREAD
  catch( boost::lock_error le ) 
    {
      sotDEBUG(1) << "Overlaps catched." << std::endl;
      if( motorCommandInit_&&robotStatePrecInit_ ) 
	robotStatePrec_ += (motorCommandPrec_*0.005);
      //      for( int i=0;i<40;++i ) 
      //	{ mc->angle[i] = robotStatePrec_(i); }
      updateMC(mc,robotStatePrec_);
    }
#endif

  previousStateSOUT.setConstant(robotStatePrec_);

  sotDEBUGOUT(5) << std::endl; 
}

void StackOfTasks::sotControlFinishing(RobotState *rs, RobotState *mc)
{ 
  // pluginState_ is either sotHOLD or sotFINISHING.
  // If sotFINISHING,
  //    - run one step of control loop and
  //    - if possible send halfsitting request to seqplay.
  // If sotHOLD,
  //    - if seqplay client is empty (?) switch to sotFINISHED
  //    - otherwise do nothing.
  sotDEBUG(15) << "--- Iter = "<<iter_ 
	       <<"-------------------------------------------------"<<endl;
  if(pluginState_ == sotFINISHING) {
    pluginState_=sotLOOP;    
    sotControlLoop(rs,mc);
    
    pluginState_=sotFINISHING;
    return ;
  }  // if(pluginState_ == sotFINISHING ))

  if(pluginState_ == sotHOLD) {
    pluginState_=sotFINISHED;
  }
  return;
}

void StackOfTasks::commandLine(const std::string&,
			       std::istringstream&,
			       std::ostream&)
{
}

void StackOfTasks::updateMC(RobotState *mc,
			    maal::boost::Vector &VectorCommand)
{
  switch(whichRobot_)
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

void StackOfTasks::updateFromMC(RobotState *mc,
				maal::boost::Vector &VectorCommand,
				unsigned int)
{
  switch(whichRobot_)
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

void StackOfTasks::updateTorque(RobotState *rs,
				maal::boost::Vector &torque)
{
  switch(whichRobot_)
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

void StackOfTasks::play( void )
{
  if( suspend_ )
    {
      suspend_ = false;
      //robotStatePrecInit_=false;
    }
}


void StackOfTasks::pause( void )
{
  if(!suspend_) 
    {
      suspend_ = true;
      //robotStatePrecInit_=false;
    }
}

void StackOfTasks::updateHRP2SmallFromMC(RobotState *mc,
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

  for(unsigned int i=0;i<numberDofs_-6;i++)
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

void StackOfTasks::updateHRP2Smalltorque(RobotState *rs,
					 maal::boost::Vector &Torque)
{
  Torque.resize(numberDofs_);
  for(unsigned int i=0;i<numberDofs_-6;i++)
    Torque(i) = rs->torque[i];
}

 
void StackOfTasks::updateHRP210SmallOldMC(RobotState *mc,
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

void StackOfTasks::updateHRP210SmallOldFromMC(RobotState *mc,
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

void StackOfTasks::updateHRP210SmallOldtorque(RobotState *rs,
					      maal::boost::Vector &Torque)
{
  unsigned int lindex = 0;
  Torque.resize(numberDofs_);
  for(unsigned int i=0;i<21;i++)
    Torque(lindex++) = rs->angle[i];
  for(unsigned int i=22;i<29;i++)
    Torque(lindex++) = rs->angle[i];
  for(unsigned int i=30;i<42;i++)
    Torque(lindex++) = rs->angle[i];
}

void StackOfTasks::updateHRP2SmallMC(RobotState *mc,
				     maal::boost::Vector &VectorCommand,
				     unsigned int ref)
{

  sotDEBUGIN(5);
  sotDEBUG(5) << "Vector.size():" <<VectorCommand.size() 
	      << " mc->angle length:" << mc->angle.length() <<endl; 

  unsigned int lindex=ref;
  for(unsigned int i=0;i<numberDofs_-6;i++)
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

