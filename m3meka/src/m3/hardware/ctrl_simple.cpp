 /*************************************************************************
 * 
 * REDWOOD CONFIDENTIAL
 * Author: Aaron Edsinger
 * __________________
 * 
 *  [2012] - [+] Redwood Robotics Incorporated 
 *  All Rights Reserved.
 * 
 * All information contained herein is, and remains
 * the property of Redwood Robotics Incorporated and its suppliers,
 * if any.  The intellectual and technical concepts contained
 * herein are proprietary to Redwood Robotics Incorporated
 * and its suppliers and may be covered by U.S. and Foreign Patents,
 * patents in process, and are protected by trade secret or copyright law.
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from Redwood Robotics Incorporated.
 */
#include "m3/hardware/ctrl_simple.h"
namespace m3{
	
using namespace m3rt;
using namespace std;


bool M3CtrlSimple::ReadConfig(const char * filename)
{
	//YAML::Node doc;
	//mReal val;
	
	if (!M3Component::ReadConfig(filename))
		return false;

	//GetYamlDoc(filename, doc);	
	
	//Misc
	doc["act_component"] >> act_name;
	NodeToTrajParam(doc["param"]["traj_current"],ParamTrajCurrent());
	NodeToTrajParam(doc["param"]["traj_theta"],ParamTrajTheta());
	NodeToTrajParam(doc["param"]["traj_torque"], ParamTrajTorque());
	
	NodeToPIDParam(doc["param"]["pid_theta"] ,ParamPidTheta());
	NodeToPIDParam(doc["param"]["pid_torque"] , ParamPidTorque());
	return true;
} // end ReadConfig

bool M3CtrlSimple::LinkDependentComponents()
{
	act = dynamic_cast<M3Actuator*>(factory->GetComponent(act_name));
	if (act==NULL)
	{
		M3_INFO("M3Actuator component %s not found for component %s\n",act_name.c_str(),GetName().c_str());
		return false;
	}
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void M3CtrlSimple::Startup()
{
	if (act!=NULL)
		SetStateSafeOp();
	else
		SetStateError();
	ctrl_mode_last = CTRL_MODE_BRAKE;
}

void M3CtrlSimple::Shutdown()
{
}

////////////////////////////////////////////////////////////////////////////////
//			STATUS


void M3CtrlSimple::StepStatus()
{ 
	if (IsStateError())
		return;
      M3ActuatorStatus *act_status  = (M3ActuatorStatus*)(act->GetStatus());
      StatusActuator()->CopyFrom(*act_status);

} // end StepStatus


////////////////////////////////////////////////////////////////////////////////
//			COMMAND





void M3CtrlSimple::StepCommand()
{
	pnt_cnt++;
	
	if (!act || IsStateSafeOp())
		return;
	
	if(IsStateError())
	{
		act->SetDesiredControlMode(ACTUATOR_MODE_OFF);
		return;
	}
	
	
	/////////////// Handle Trajectories ////////////////////
	
	mReal	desired_theta;
	mReal	desired_current;
	mReal	desired_torque;
	mReal	output;
	
	//Trajectories
	mReal dt			= GetTimestamp()/1000000.0;//seconds

	mReal t_theta	= sin( 2*M_PI * dt * ParamTrajTheta()->freq());
	mReal t_current	= sin( 2*M_PI * dt * ParamTrajCurrent()->freq());
	mReal t_torque	= sin( 2*M_PI * dt * ParamTrajTorque()->freq());
	
	// Grab the commmand values
	desired_theta		= command.desired_theta();
	desired_current		= command.desired_current();
	desired_torque		= command.desired_torque();
	
	if (command.traj_mode() ==TRAJ_SQUARE && command.ctrl_mode()==CTRL_MODE_CURRENT)
	{
		if (t_current>0)
		    desired_current = ParamTrajCurrent()->zero()+ParamTrajCurrent()->amp();
		else
		    desired_current = ParamTrajCurrent()->zero()-ParamTrajCurrent()->amp();
	}
	if (command.traj_mode() ==TRAJ_SINE && command.ctrl_mode()==CTRL_MODE_CURRENT)
	{
	  desired_current	 = ParamTrajCurrent()->zero()+ParamTrajCurrent()->amp()*t_current;
	}
	
	if (command.traj_mode() ==TRAJ_SQUARE && 
	  (command.ctrl_mode()==CTRL_MODE_THETA ||command.ctrl_mode()==CTRL_MODE_THETA_IMP))
	{
		if (t_theta>0)
			desired_theta = ParamTrajTheta()->zero()+ParamTrajTheta()->amp();
		else
			desired_theta = ParamTrajTheta()->zero()-ParamTrajTheta()->amp();
	}
	if (command.traj_mode() ==TRAJ_SINE && 
	  (command.ctrl_mode()==CTRL_MODE_THETA ||command.ctrl_mode()==CTRL_MODE_THETA_IMP))
	{
		desired_theta = ParamTrajTheta()->zero()+ParamTrajTheta()->amp()*t_theta;
	}
	
	if (command.traj_mode() ==TRAJ_SQUARE && 
	  (command.ctrl_mode()==CTRL_MODE_TORQUE ||command.ctrl_mode()==CTRL_MODE_TORQUE_GC))
	{
		if (t_torque>0)
			desired_torque = ParamTrajTorque()->zero()+ParamTrajTorque()->amp();
		else
			desired_torque = ParamTrajTorque()->zero()-ParamTrajTorque()->amp();
	}
	if (command.traj_mode() ==TRAJ_SINE && 
	  (command.ctrl_mode()==CTRL_MODE_TORQUE ||command.ctrl_mode()==CTRL_MODE_TORQUE_GC))
	{
		  desired_torque = ParamTrajTorque()->zero()+ParamTrajTorque()->amp()*t_torque;
	}

	/////////////////// Command Actuator /////////////////////////
	//Reset
	StatusCommand()->set_current(0);
	StatusCommand()->set_torque(0);
	StatusCommand()->set_theta(0);
	
	if (command.ctrl_mode()== CTRL_MODE_TORQUE_GC)
	  desired_torque -= status.torque_gravity();
	
	if ((command.ctrl_mode() != ctrl_mode_last) || !act->IsMotorPowerSlewedOn())
	{
		ResetIntegrators();
		
	}
	ctrl_mode_last = command.ctrl_mode();
	
	//Handle Inner Control Loops
	switch(command.ctrl_mode())
	{
		case CTRL_MODE_CURRENT:
			if (act->GetAmpControlInput() != ACTUATOR_INPUT_CURRENT)
				break;
			act->SetDesiredControlMode(ACTUATOR_MODE_CURRENT);
			act->SetDesiredCurrent(desired_current);
			// update personal status
			StatusCommand()->set_current(desired_current);
			break;
		case CTRL_MODE_TORQUE:
		//case CTRL_MODE_TORQUE_GC:
			//PID
			if (act->GetAmpControlInput() != ACTUATOR_INPUT_CURRENT)
				break;
			desired_current = pid_torque.Step(GetJointTorque(),
						  GetJointTorqueDot(),
						  desired_torque,
						  ParamPidTorque()->k_p(), 
						  ParamPidTorque()->k_i(),
						  ParamPidTorque()->k_d(),
						  ParamPidTorque()->k_i_limit(),
						  ParamPidTorque()->k_i_range());
						  
			
			act->SetDesiredControlMode(ACTUATOR_MODE_CURRENT);
			act->SetDesiredCurrent(desired_current);
			// update personal status
			StatusCommand()->set_current(desired_current);
			StatusCommand()->set_torque(desired_torque);
			break;
		
		case CTRL_MODE_THETA:
			//PID
			output = pid_theta.Step(GetJointTheta(),
									GetJointThetaDot(),
									desired_theta,
									ParamPidTheta()->k_p(), 
									ParamPidTheta()->k_i(),
									ParamPidTheta()->k_d(),
									ParamPidTheta()->k_i_limit(),
									ParamPidTheta()->k_i_range());
			
		
			if (act->GetAmpControlInput() == ACTUATOR_INPUT_PWM)
			{
				act->SetDesiredControlMode(ACTUATOR_MODE_PWM);
				act->SetDesiredPwm((int)output);
				
				// update personal status
				StatusCommand()->set_pwm(output);
				StatusCommand()->set_theta(desired_theta);
			}
			else if (act->GetAmpControlInput() == ACTUATOR_INPUT_CURRENT)
			{
				act->SetDesiredControlMode(ACTUATOR_MODE_CURRENT);
				act->SetDesiredCurrent(output);
				// update personal status
				StatusCommand()->set_current(output);
				StatusCommand()->set_theta(desired_theta);
			}
			break;
		case CTRL_MODE_BRAKE:
			act->SetDesiredControlMode(ACTUATOR_MODE_BRAKE);
			break;
		case CTRL_MODE_THETA_IMP: //Not yet implemented
		case CTRL_MODE_OFF:
		default:
			act->SetDesiredControlMode(ACTUATOR_MODE_OFF);
			break;


	}

}


} // end StepCommand
