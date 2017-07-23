/* 
M3 -- Meka Robotics Robot Components
Copyright (c) 2010 Meka Robotics
Author: edsinger@mekabot.com (Aaron Edsinger)

M3 is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

M3 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with M3.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "m3/hardware/actuator_virtual.h"
#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/component_factory.h"


namespace m3{
	
using namespace m3rt;
using namespace std;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void M3ActuatorVirtual::Startup()
{
	SetStateSafeOp();
	status.set_torque(0.0);
	status.set_torquedot(0.0);
}

/*void M3ActuatorVirtual::Shutdown()
{

}*/

bool M3ActuatorVirtual::ReadConfig(const char * filename)
{
	if (!M3Actuator::ReadConfig(filename))
		return false;
	doc["joint_component"] >> jnt_name;
	if(!torque_df.ReadConfig(doc["param"]["angle_df"])) // A.H : now allow to modify it online
		{
	if(!torque_df.ReadConfig( doc["calib"]["angle_df"])){
					M3_ERR("torque_df config error (keys missing)\n");
					return false;
					
				}
	}
	mReal val=0.0;
	try
		{
			doc["param"]["k_qdot_virtual"] >> val;
			param.set_k_qdot_virtual(val);
		} catch(...) 
		{
			param.set_k_qdot_virtual(0.0);
		} 
	try
		{
			doc["param"]["k_qdotdot_virtual"] >> val;
			param.set_k_qdotdot_virtual(val);
		} catch(...) 
		{
			param.set_k_qdotdot_virtual(0.0);
		} 
	return true;
}

bool M3ActuatorVirtual::LinkDependentComponents()
{
	joint=reinterpret_cast<M3Joint *>( factory->GetComponent(jnt_name));
	if (joint==NULL)
	{
		M3_INFO("M3Joint component %s not found for component %s. Proceeding without it.\n",jnt_name.c_str(),GetName().c_str());
		return false;
	}
	try{
	joint->GetConfig()["param"]["max_q_slew_rate"] >> max_q_slew_rate;
	}catch(...){
	  max_q_slew_rate = 25.0;
	}

	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void M3ActuatorVirtual::StepStatus()
{
	pnt_cnt++;
	
	if (IsStateError())
		return;

	
	status.set_flags(ACTUATOR_EC_FLAG_QEI_CALIBRATED);


	if (joint != NULL)
	{
		M3Transmission * t=joint->GetTransmission();
		if (t!=NULL)
		{
			this->StepFilterParam();
			torque_df.Step((-this->joint->GetTorqueGravity())/1000.0,status.torquedot());
			status.set_torque(torque_df.GetX());
			status.set_torquedot(torque_df.GetXDot());
			// A.H test : torque and not torquedot (I don't care about torquedot)
			mReal th = t->GetThetaDesJointDeg() +param.k_qdot_virtual()*status.thetadot()+param.k_qdotdot_virtual()*status.thetadotdot();
			angle_df.Step(slew.Step(th,max_q_slew_rate),status.thetadot()); //Note: should be GetThetaDesSensorDeg, not working. this OK so long as all angle sensors are collocated 1:1
			status.set_theta(angle_df.GetTheta());
			status.set_thetadot(angle_df.GetThetaDot());
			status.set_thetadotdot(angle_df.GetThetaDotDot());
			status.set_amp_temp(25.0);
			//status.set_current(0.0);
			status.set_motor_temp(25.0);
		}
		else
			M3_INFO("No transmission found for %s\n",joint->GetName().c_str());
	}

}

void M3ActuatorVirtual::StepCommand()
{

}

}
