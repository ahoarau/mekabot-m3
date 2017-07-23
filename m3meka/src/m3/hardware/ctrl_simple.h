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
 

#ifndef M3_CTRL_SIMPLE_H
#define M3_CTRL_SIMPLE_H

#include <cmath>
#include "m3rt/base/toolbox.h"
#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/component_factory.h"
#include <google/protobuf/message.h>
#include "m3rt/base/component.h"
#include "m3/toolbox/toolbox.h"
#include "m3/toolbox/dfilter.h"
#include "m3/hardware/ctrl_simple.pb.h"
#include "m3/hardware/actuator.h"
#include "m3/hardware/actuator.pb.h"

namespace m3
{
	using namespace std;
	using namespace m3rt;
	
class M3CtrlSimple: public  m3rt::M3Component
{
	public:
		M3CtrlSimple(): m3rt::M3Component(CONTROL_PRIORITY),	pnt_cnt(0),act(NULL)
		{
			RegisterVersion("default",DEFAULT);	
		}
		
	//Setters
		void SetDesiredControlMode(CTRL_SIMPLE_MODE x){command.set_ctrl_mode(x);}
		void SetDesiredCurrent(mReal i){command.set_desired_current(i);}
		void SetDesiredTheta(mReal q){command.set_desired_theta(q);}
		void SetDesiredTorque(mReal tq){command.set_desired_torque(tq);}
		void SetDesiredStiffness(mReal s){command.set_desired_stiffness(s);}
		void SetTorqueGravity(mReal tq){status.set_torque_gravity(tq);}
		
	//Access to Status Messages
		M3BaseStatus *			StatusBase()	{return status.mutable_base();}
		M3CtrlSimpleStatusCommand *	StatusCommand()	{return status.mutable_command();}
		M3ActuatorStatus *		StatusActuator(){return status.mutable_actuator();}
		
	//Access to Traj Params
		M3ParamTrajectory * 	ParamTrajCurrent()	{return param.mutable_traj_current();}
		M3ParamTrajectory * 	ParamTrajTheta()	{return param.mutable_traj_theta();}
		M3ParamTrajectory * 	ParamTrajTorque()	{return param.mutable_traj_torque();}
		M3ParamPID *		ParamPidTheta()		{return param.mutable_pid_theta();}
		M3ParamPID * 		ParamPidTorque()	{return param.mutable_pid_torque();}

	//Conversions
		virtual	bool	IsMotorPowerOn()		{if (!GetActuator()) return false; return act->IsMotorPowerOn();}
		M3Actuator * GetActuator()	{return act;}

	// getters
		mReal	GetJointTheta()			{return act->GetThetaRad();}
		mReal	GetJointThetaDot()		{return act->GetThetaDotRad();}
		mReal	GetJointTorque()		{return act->GetTorque();}
		mReal	GetJointTorqueDot()		{return act->GetTorqueDot();}
		
		int64_t GetTimestamp()			{return GetBaseStatus()->timestamp();}
		
		void ResetIntegrators(){pid_torque.ResetIntegrator();pid_theta.ResetIntegrator();}
		
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}

	protected:

		// required:
		enum{DEFAULT};
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		bool LinkDependentComponents();
		bool ReadConfig(const char * filename);
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
		
		M3CtrlSimpleStatus	status;
		M3CtrlSimpleCommand	command;
		M3CtrlSimpleParam	param;
		
		
		M3PID	pid_theta;
		M3PID	pid_torque;		
		
		M3Actuator * act;
		string	act_name;
		CTRL_SIMPLE_MODE ctrl_mode_last;
		
		int pnt_cnt;
private:
	void NodeToTrajParam(const YAML::Node& node, M3ParamTrajectory* traj)
{
#ifdef YAMLCPP_03
	mReal tmp;
	node["freq"] >> tmp; 		traj->set_freq(tmp);
	node["amp"] >> tmp;			traj->set_amp(tmp);
	node["zero"] >> tmp;		traj->set_zero(tmp);
#else
	traj->set_freq(node["freq"].as<mReal>());
	traj->set_amp(node["amp"].as<mReal>());
	traj->set_zero(node["zero"].as<mReal>());
#endif
}
void NodeToPIDParam(const YAML::Node& node, M3ParamPID* pid)
{
#ifdef YAMLCPP_03

	mReal val;
	node["k_p"] >> val;			pid->set_k_p(val);
	node["k_i"] >> val;			pid->set_k_i(val);
	node["k_d"] >> val;			pid->set_k_d(val);
	node["k_i_limit"] >> val;	pid->set_k_i_limit(val);
	node["k_i_range"] >> val;	pid->set_k_i_range(val);
#else
	pid->set_k_p(node["k_p"].as<mReal>());
	pid->set_k_i(node["k_i"].as<mReal>());
	pid->set_k_d(node["k_d"].as<mReal>());
	pid->set_k_i_limit(node["k_i_limit"].as<mReal>());
	pid->set_k_i_range(node["k_i_range"].as<mReal>());
#endif
}
};

}

#endif


