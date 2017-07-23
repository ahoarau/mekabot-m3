#include "Pcv.h"

using namespace std;

Ctrl::Ctrl(int step_freq, vector<Float> gx_init, Pcv_Params params_in)
  : Jt(NULL),J(NULL),motor_tq(NULL),
  Lambda(3,3),Mu(3),cf(3),cxdd(3),fcxdd(3),cgxdd(3),C(8,3),qd_bar(NULL),qd_null(8),
  tq(NULL),tqE(8),rot(3,3),E(NUM_TRUSS_LINKS,8),dac_ticks(NULL),e(NULL),
  Et(8,NUM_TRUSS_LINKS),rtE(NUM_TRUSS_LINKS),tE(NUM_TRUSS_LINKS),ctE(NUM_TRUSS_LINKS),
  params(params_in),veh(NULL),q(NULL),qd(NULL),rxd(3),rgxd(3),x(3),xd(3),gx(3),gxd(3),tmp_cnt(0),
  gxdd(3),dx(3),dxd(3),dxdd(3),cxd(3),first_time(true),traj(NULL),motor_vel(NULL)
{    
  ////////////////////////////////
  /* Init non-yaml variables */
  //////////////////////////////
  ENC_REV2CNT = params.caster.ENC_REV2CNT;
  hw_freq = step_freq;  
    hw_period = 1.0/hw_freq;
            
    J = new PrMatrix(3, params.geometry.num_casters*2);
    Jt = new PrMatrix(params.geometry.num_casters*2, 3);
    tq = new PrVector(params.geometry.num_casters*2);
    motor_tq = new PrVector(params.geometry.num_casters*2);
    qd = new PrVector(params.geometry.num_casters*2);
    q = new PrVector(params.geometry.num_casters*2);    
    qd_bar = new PrVector(params.geometry.num_casters*2);
    e = new PrVector(params.geometry.num_casters*2);
    dac_ticks = new PrVector(params.geometry.num_casters*2);
    motor_vel = new PrVector(params.geometry.num_casters*2);
    // Zero the Base (ZERO HAPPENS UPON Vehicle OBJ CREATION)
    veh = Vehicle::HandleGet(params);    
    
    veh->Add_Solid(  0.000,  0.000,  params_in.mass.mass,params_in.mass.inertia_flat);
    
    for (int i = 0; i < params.geometry.num_casters; i++)
    {
      veh->Add_Caster( params.geometry.caster_placement[i].x,
		       params.geometry.caster_placement[i].y,
		       params.geometry.caster_placement[i].angle,
		       params.geometry.steering_offsets[i]);			   
    }        
    
    tq->zero();
    motor_tq->zero();
    rot.at(2,2) = 1;    
  
    for (int i = 0; i < 3; i++)
    {
      gx[i] = gx_init[i];
      old_traj_goal[i] = 0;
    }
  // INITIALIZE FILTERS
    
    //veh->JointRad(q);
    if(params.filter.FQD) Fqd.D_LowPass(*q, hw_freq, params.filter.FQD);
    //if(params.filter.FQD) Fqd.D_LowPass3(q, hw_freq, params.filter.FQD);
    else    Fqd.D(*q, hw_freq);
    
    //veh->JointRad(q);    
    /*for (int i = 0; i < 8; i++)
    {
      Fqd_meka[i] = new M3DFilter2();
      if(params.filter.FQD) 
	Fqd_meka[i]->Diff_Butterworth_Filter(3, params.filter.FQD,hw_period);
    }*/
        
    rgxd.zero();
    if(params.filter.FGXD) Fgxd.LowPass(rgxd, hw_freq, params.filter.FGXD);
    else     Fgxd.Unity();
    
    cxdd.zero();
    if(params.filter.FCXDD) Fcxdd.LowPass(cxdd, hw_freq, params.filter.FCXDD);
    else      Fcxdd.Unity();
    
    rtE.zero();
     if(params.filter.FTE) FtE.LowPass(rtE, hw_freq, params.filter.FTE);
     else    FtE.Unity();

    rxd.zero();
    if(params.filter.FXD) Fxd.LowPass(rxd, hw_freq, params.filter.FXD);
    else    Fxd.Unity();    
    
    old_traj_mode = TRAJ_M_OFF;
    
    // INTIALIZE traj (GLOBAL)
    traj = new Traj3(hw_freq);
    
    /*ax_ = 0.25;
    aw_ = 0.2 * REV2RAD;
    vx_ = 0.5;
    vw_ = 0.15 * REV2RAD;        */
}

Ctrl::~Ctrl()
{  
  if (veh != NULL)
    delete veh;
  veh = NULL;
  /*for (int i = 0; i < 8; i++)
  {
    if (Fqd_meka[i] != NULL)
      delete Fqd_meka[i];
    Fqd_meka[i] = NULL;
  }*/
  if (traj != NULL)
    delete traj;
  traj = NULL;
  if (J != NULL)
  {
      delete J;
      J = NULL;
  }
  if (Jt != NULL)
  {
      delete Jt;
      Jt = NULL;
  }
  if (motor_tq != NULL)
  {
      delete motor_tq;
      motor_tq = NULL;
  }
  if (tq != NULL)
  {
      delete tq;
      tq = NULL;
  }
  if (q != NULL)
  {
      delete q;
      q = NULL;
  }
  if (qd != NULL)
  {
      delete qd;
      qd = NULL;
  } 
  if (qd_bar != NULL)
  {
      delete qd_bar;
      qd_bar = NULL;
  }
  if (e != NULL)
  {
      delete e;
      e = NULL;
  }
  if (dac_ticks != NULL)
  {
      delete dac_ticks;
      dac_ticks = NULL;
  }    
}

void Ctrl::TrajInit(Pcv_Command pcv_cmd)
{
  PrVector destPos(3);  
  /*ax_ = 0.5;
    aw_ = 0.3 * REV2RAD;
    vx_ = 0.5;
    vw_ = 0.10 * REV2RAD;*/
   /*ax_ = 0.5;
    aw_ = 0.2 * REV2RAD;
    vx_ = 0.5;
    vw_ = 0.15 * REV2RAD;*/
  /*ax_ = 0.50; // linear acceleration max
  aw_ = 0.30 * REV2RAD; // angular acc max
  vx_ = 0.70; // linear vel max
  vw_ = 0.15 * REV2RAD; // ang vel max*/
  traj->curPos( gx  );
  traj->curVel( gxd );
  traj->accel(  CLAMP(pcv_cmd.max_linear_acceleration, 0, params.kinematics.abs_max_linear_acceleration),
		CLAMP(pcv_cmd.max_rotation_acceleration, 0, params.kinematics.abs_max_rotation_acceleration) );
  traj->maxVel( CLAMP(pcv_cmd.max_linear_velocity, 0, params.kinematics.abs_max_linear_velocity),		      
		CLAMP(pcv_cmd.max_rotation_velocity, 0,  params.kinematics.abs_max_rotation_velocity) );
  destPos[0] = pcv_cmd.traj_goal[0];
  destPos[1] = pcv_cmd.traj_goal[1];
  destPos[2] = pcv_cmd.traj_goal[2];
  printf("init %f,%f,%f\n",destPos[0],destPos[1],destPos[2]);
  traj->dest2(destPos);
    
}


Pcv_Status Ctrl::Step(Pcv_Command pcv_cmd)
{
  cmd = pcv_cmd;
  status.traj_goal_reached = false;
  
  switch( cmd.trajMode_ )
  {
    case TRAJ_M_CART_LOCAL:    
      for (int i = 0; i < 3; i++)
      {
	dx[i] = cmd.local_position_desired[i]; 
	dxd[i] = cmd.local_velocity_desired[i];
	dxdd[i] = cmd.local_acceleration_desired[i];
      }
      break;
    case TRAJ_M_CART_GLOB:    
      for (int i = 0; i < 3; i++)
      {
	dx[i] = cmd.global_position_desired[i]; 
	dxd[i] = cmd.global_velocity_desired[i];
	dxdd[i] = cmd.global_acceleration_desired[i];
      }
      break;
    case TRAJ_M_JOYV:
	if (old_traj_mode != TRAJ_M_JOYV)
	  JoystickInit(cmd);
      StepJoystick(cmd);   
      break;
    case TRAJ_M_GOAL:  
      if (old_traj_mode != TRAJ_M_GOAL || old_traj_goal[0] != cmd.traj_goal[0] ||
	 old_traj_goal[1] != cmd.traj_goal[1] ||  old_traj_goal[2] != cmd.traj_goal[2])   
      {
	TrajInit(cmd);
	for (int i = 0; i < 3; i++)
	  old_traj_goal[i] = cmd.traj_goal[i];
      }
      if( traj->get(dx, dxd, dxdd) == 0 ) 
	  status.traj_goal_reached = true; 
      
      break;
    case TRAJ_M_JOINT_TORQUE:
    case TRAJ_M_OFF:
    default:
      dx.zero();
      dxd.zero();
      dxdd.zero();
      break;
  }

  // BEGIN ODOMETRY SECTION
  if (cmd.use_angles_instead_of_ticks)
  {
    for (int i = 0; i < veh->GetNumCasters()*2; i++)      
      cmd.enc_ticks[i] = int(round(ENC_RAD2CNT*cmd.motor_angles_rad[i]));
  }
    
    for (int i = 0; i < veh->GetNumCasters()*2; i++)      
      (*e)[i] = cmd.enc_ticks[i];
    
    veh->JointRad(*q,*e);   
  
  if (cmd.adjust_global_position){
      for (int i = 0; i < 3; i++)
	gx[i] = cmd.global_position[i];       
  }  
  if (cmd.adjust_local_position){
      for (int i = 0; i < 3; i++)
	x[i] = cmd.local_position[i];        
  }
  if (cmd.adjust_local_position || cmd.adjust_global_position){
    
    Fqd.D_LowPass_Reset(*q);    
          
    rgxd.zero();
    if (params.filter.FGXD)
      Fgxd.LowPass_Reset(rgxd);
    rtE.zero();
    if (params.filter.FTE)
      FtE.LowPass_Reset(rtE);
    cxdd.zero();
    if (params.filter.FCXDD)
      Fcxdd.LowPass_Reset(cxdd);
    rxd.zero();
    if (params.filter.FXD)
      Fxd.LowPass_Reset(rxd);      
    //for (int i = 0; i < 8; i++)
      //Fqd_meka[i]->Clear();
    first_time = true;
  }            
      
     Fqd.Filt(*q,*qd);     // JOINT SPEEDS
     
     if ( cmd.use_external_motor_velocities )
     {
       for (int i = 0; i < veh->GetNumCasters()*2; i++)
	  (*motor_vel)[i] = cmd.motor_velocities_rad[i];
       veh->JointVel(*qd,*motor_vel); 
     }
     
     //for (int i = 0; i < 8; i++)
       //qd[i] = Fqd_meka[i]->Step(q[i]);  //TODO: find out why this is lagging server so much?
     
     // FIND LOCAL OPERATIONAL SPEEDS
      veh->Fill_Jcp(*J);       // NOTE: VIA CONTACT POINTS
      rxd = (*J) * (*qd);           // RAW LOCAL OP SPEEDS
    
      x += rxd * hw_period;   // "LOCAL" COORDS
      Fxd.Filt( rxd, xd );    // LOCAL SPEED (ALSO FOR DYN)          
      //x += xd * hw_period;   // "LOCAL" COORDS
      
    // DELTA: MAP LOCAL --> GLOBAL COORDS
      heading = gx[2]+ 0.5*rxd[2]*hw_period; // USE RAW_xd
      rot.at(0,0) =  cos(heading);
      rot.at(0,1) = -sin(heading);
      rot.at(1,0) =  sin(heading);
      rot.at(1,1) =  cos(heading);
      rot.multiply(rxd,rgxd);               // GET RAW_gxd
    
    // INTEGRATION TO GLOBAL COORDS      
      gx += rgxd * hw_period;

      Fgxd.Filt(rgxd,gxd);     // FILTERED GLOBAL VELOCITY
      //gx += gxd * hw_period;

// END ODOMETRY SECTION

// BEGIN CONTROL FORCE SECTION

        // GLOBAL CONTROL
      if( cmd.ctrlFrame_ == CTRL_F_GLOBAL )
      { 
        cgxdd[0] = (dx[0]-gx[0])*params.gains.KPx_ + (dxd[0]-gxd[0])*params.gains.KVx_;
        cgxdd[1] = (dx[1]-gx[1])*params.gains.KPx_ + (dxd[1]-gxd[1])*params.gains.KVx_;
        cgxdd[2] = (dx[2]-gx[2])*params.gains.KPa_ + (dxd[2]-gxd[2])*params.gains.KVa_;

        if( cmd.trajMode_ == TRAJ_M_ACCEL )
          cgxdd.zero();

        if( cmd.accel_FF_ == true )
          cgxdd += dxdd;

        // GLOBAL OP SPACE ADJUSTMENTS (PUT HACKs HERE)
//         if( mi->trajMode_ == TRAJ_M_PACE )
//           cgxdd[2]=0;

        // MAP GLOBAL COMMAND TO LOCAL COORDS
        rot.at(0,1) *= -1.0;
        rot.at(1,0) *= -1.0;
        rot.multiply(cgxdd,cxdd);
      }
      else  // LOCAL CONTROL
      { 	
	Float xd_err = dxd[0]-xd[0];
	Float yd_err = dxd[1]-xd[1];	
        cxdd[0] = (dx[0]-x[0])*params.gains.KPx_ + (xd_err)*params.gains.KVx_;	
        cxdd[1] = (dx[1]-x[1])*params.gains.KPx_ + (yd_err)*params.gains.KVx_;
        cxdd[2] = (dx[2]-x[2])*params.gains.KPa_ + (dxd[2]-xd[2])*params.gains.KVa_;

        if( cmd.trajMode_ == TRAJ_M_ACCEL )
          cxdd.zero();

        if( cmd.accel_FF_ == true )
          cxdd += dxdd;

        // LOCAL OP SPACE ADJUSTMENTS (PUT HACKs HERE)
                //cxdd[2]=0;
      }

    // NULL MODE OVERRIDE (CLOBBER HERE, FOR SAFETY)
      if( cmd.trajMode_ == TRAJ_M_NULL )
        cxdd.zero();

      //Fcxdd.Filt(cxdd,fcxdd);
      //cxdd = fcxdd;            // replace w/ filtered

// END CONTROL FORCE SECTION

// BEGIN OUTPUT TORQUE SECTION

    // DO DYNAMICS
      veh->Fill_C(C);         // USE CONSISTENT SPEEDS
      C.multiply(xd,*qd_bar);  // IGNORE SLIP FOR DYNAMICS
      veh->Dyn(*qd_bar, xd[2]);// FILL Lambda & Mu
      Lambda = veh->Lambda;
      Mu     = veh->Mu;                 
      
    // GET MAPPING OP FORCE -> JOINT TQ
      J->transpose(*Jt); // MOTOR TQ's w/ MIN CONTACT FORCES 

      switch( cmd.controlMode_ )
      {
      case CTRL_LM:
          cf = (Lambda*cxdd);
          *tq = *Jt *(cf + Mu);
        break;

      case CTRL_L:	  
          cf = (Lambda*cxdd);
          *tq = *Jt * cf;
        break;

      case CTRL_L_CONST: // USE AVERAGE MASS PROPERTIES
          cf[0] = params.mass.mass * cxdd[0];  // AVERAGE MASS
          cf[1] = params.mass.mass * cxdd[1];
          cf[2] =   params.mass.inertia_flat * cxdd[2];  // AVERAGE INERTIA
          *tq = *Jt * cf ;
        break;
	
      case CTRL_CF_CMD: // USE external CF command
          cf[0] = cmd.opspace_force_desired[0];
          cf[1] = cmd.opspace_force_desired[1];
          cf[2] =   cmd.opspace_force_desired[2];
          *tq = *Jt * cf ;
        break;

      case CTRL_TEST:
          tq->zero();
          (*tq)[2] = 2.0;
        break;      

      default:  
	tq->zero();
	cf.zero();
        break;
      }  // end switch controlMode

   

    // INTERNAL FORCE COMPUTATION
      // TODO: enable for variable caster numbers, default in 4 now
      if( cmd.internalForce_ == true && veh->GetNumCasters()==4)
      { 
        veh->Fill_E_q( E );
        E.transpose( Et );	
        // PROJECT TO 'E'-SPACE AND THEN BACK TO JT-SPACE
        qd_null = (*qd) - (*qd_bar);  // NULL SPACE WHEEL SPEEDS	
        E.multiply(qd_null,rtE);    // RAW SLIP (INTERNAL VELs)
        FtE.Filt(rtE,tE);       // FILTER INTERNAL VELs	
        tE.multiply(-params.gains.KpE_,ctE); // CONTROL FORCES to RESIST SLIP
        Et.multiply(ctE,tqE);   // MAP TO JOINT TORQUES
        *tq += tqE;
      }
            
      // boost steer torque to account for friction in drive train
      for (int i = 0; i < veh->GetNumCasters(); i++)
	 (*tq)[i*2] *= params.gains.KpC_;
      
      if (cmd.trajMode_ == TRAJ_M_JOINT_TORQUE)
      {
	for (int i = 0; i < veh->GetNumCasters(); i++)
	{
	    (*tq)[i*2] = cmd.steer_torque_desired_Nm[i];
	    (*tq)[i*2+1] = cmd.roll_torque_desired_Nm[i];
	}	
      }

// OUTPUT TORQUE TO JOINTS
      veh->JointTq(*tq, *motor_tq);
      veh->MotorTq(*motor_tq, *dac_ticks);
//extra
//veh->JtTq2MotAmp(tq,motAmp); // extra
//tqI = Jt *(Lambda*dxdd + Mu);

  old_traj_mode = cmd.trajMode_;
// END OUTPUT TORQUE SECTION
  for (int i = 0; i < veh->GetNumCasters()*2; i++)
  {    
    status.motor_torque_Nm[i] = (*motor_tq)[i];        
    status.dac_ticks[i] = (*dac_ticks)[i];
  }
  for (int i = 0; i < veh->GetNumCasters(); i++)
  {
   status.steer_angle_rad[i] = (*q)[i*2];
   status.roll_angle_rad[i] = (*q)[i*2+1];
   status.steer_velocity_rad[i] = (*qd)[i*2];
   status.roll_velocity_rad[i] = (*qd)[i*2+1];
   status.steer_torque_Nm[i] = (*tq)[i*2]; 
   status.roll_torque_Nm[i] = (*tq)[i*2+1];
   status.steer_torque_internal[i] = (tqE)[i*2]; 
   status.roll_torque_internal[i] = (tqE)[i*2+1]; 
  }
  
  for (int i = 0; i < 3; i++)
  {
    status.local_force[i] = cf[i];
    status.local_acceleration[i] = cxdd[i];
    status.global_position[i] = gx[i];
    status.global_velocity[i] = gxd[i];
    status.local_velocity[i] = xd[i];
    status.local_position[i] = x[i];
    status.position_desired[i] = dx[i];
    status.velocity_desired[i] = dxd[i];
    status.acceleration_desired[i] = dxdd[i];
    if( cmd.ctrlFrame_ == CTRL_F_GLOBAL )
    {
      status.position_error[i] = dx[i]-gx[i];
      status.velocity_error[i] = dxd[i]-gxd[i];
    } else {
      status.position_error[i] = dx[i]-x[i];
      status.velocity_error[i] = dxd[i]-xd[i];
    }
  }
  
  for (int i = 0; i < 6; i++)
      status.truss_vel[i] = tE[i];
  
  //printf("g:%f,%f,%f\n",gx[0],gx[1],gx[2]);
  //printf("x:%f,%f,%f\n",x[0],x[1],x[2]);
  

  return status;
     
}

void Ctrl::StepJoystick(Pcv_Command cmd)
{
  double jx = cmd.joystick_x;
  double jy = cmd.joystick_y;
  double jyaw = cmd.joystick_yaw;
  int jb = cmd.joystick_button;
  //static float jx=0,jy=0;
    //static int jb=0,jb_old=-1;
    static int jb_old=-1;
    static Float funOff = 0;
    static Float funX = 0;

    if( jb != jb_old )
      switch( jb )
      { case 0:
	case 1:		  
	  cmd.ctrlFrame_ = CTRL_F_LOCAL;
	  traj->curPos( x  );
	  traj->curVel( xd );
	  break;
	/*case 2:
	  cmd.ctrlFrame_ = CTRL_F_GLOBAL;
	  traj->curPos( gx  );
	  traj->curVel( gxd );
	  funX = gx[2] + funOff;
	  break;*/
      }
    switch(jb)
    { case 0:
	cxd[0] =  jx;
	cxd[1] =  jy;
	cxd[2] =  jyaw;
	traj->get_uV( cxd, dx, dxd, dxdd );	
	//MSG("%f, %f, %f, %f, %f, %f\n",jx,jy,jyaw,dx[0],dxd[0],dxdd[0]);
	break;
    /*case 1:
      cxd[0] = jx*cos(funOff);
      cxd[1] = -jx*sin(funOff);
      cxd[2] = jyaw;
      traj->get_uV( cxd, dx, dxd, dxdd );
      break;
    case 2:
      funOff -= rxd[2]/hw_freq;
      cxd[0] = jx*cos(funX);
      cxd[1] =  -jx*sin(funX);
      cxd[2] = jyaw;
  //                 cxd[0] = -jy*sin(funOff) + 0.5*jy*rxd[2]*cos(funOff);
  //                 cxd[1] =  jy*cos(funOff) + 0.5*jy*rxd[2]*sin(funOff);
      traj->get_uV( cxd, dx, dxd, dxdd );
      break;*/
    default:
      cxd.zero();
      traj->get_uV( cxd, dx, dxd, dxdd );
      break;
    }
    jb_old = jb; // AGE BUTTON VALUE
}

void Ctrl::JoystickInit(Pcv_Command pcv_cmd)
{
  /*ax_ = 0.50; // linear acceleration max
      aw_ = 0.30 * REV2RAD; // angular acc max
      vx_ = 0.70; // linear vel max
      vw_ = 0.15 * REV2RAD; // ang vel max*/
     /* ax_ = 0.3;
    aw_ = 0.1 * REV2RAD;
    vx_ = 0.5;
    vw_ = 0.15 * REV2RAD;*/
     /*ax_ = 0.2;
    aw_ = 0.05 * REV2RAD;
    vx_ = 0.3;
    vw_ = 0.07 * REV2RAD;*/
    x.zero();  // ZERO THE LOCAL COORDS
    traj->curPos( x  );
    traj->curVel( xd );
    traj->accel(  CLAMP(pcv_cmd.max_linear_acceleration, 0, params.kinematics.abs_max_linear_acceleration),
		CLAMP(pcv_cmd.max_rotation_acceleration, 0, params.kinematics.abs_max_rotation_acceleration) );
    traj->maxVel( CLAMP(pcv_cmd.max_linear_velocity, 0, params.kinematics.abs_max_linear_velocity),		      
		CLAMP(pcv_cmd.max_rotation_velocity, 0,  params.kinematics.abs_max_rotation_velocity) );
}