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

#ifndef M3_ACTUATOR_VIRTUAL_H
#define M3_ACTUATOR_VIRTUAL_H

#include "m3/hardware/actuator.h"
#include "m3/hardware/joint.h"
#include "m3/hardware/joint_virtual.h"
namespace m3
{
  /////////////////////////////////////////////////////////////////////////

class M3ActuatorVirtual : public m3::M3Actuator
{
	public:
		M3ActuatorVirtual():joint(0){}	
		virtual bool IsMotorPowerOn(){return true;}
		virtual bool IsMotorPowerSlewedOn(){return true;}
	protected:
		void Startup();
		void StepStatus();
		void StepCommand();
		bool LinkDependentComponents();
		int GetTicks() {return 33;}
		bool IsEncoderCalibrated(){return true;}
		void SetLimitSwitchNegZeroEncoder(){}
		void ClrLimitSwitchNegZeroEncoder(){}
		bool ReadConfig(const char * filename);
		std::string jnt_name	;
		m3::M3Joint * joint;
		M3JointFilter torque_df; // Just for simulated torque, usign angle_df params
		M3TimeSlew slew;
		double max_q_slew_rate;
		
};


}

#endif


