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

#include "m3/hardware/joint_zlift_virtual.h"

namespace m3
{
	
using namespace m3rt;
using namespace std;
//A.H

#define MN_PER_KG 9.80665*1000.0
void M3JointZLiftVirtual::Startup()
{
  M3Joint::Startup();
  GetTransmission()->SetThetaDesJointDeg(19800);
}
void M3JointZLiftVirtual::StepCommand()
{
	mReal tq_g_mNm = -1.0*GetPayloadMass()*MN_PER_KG/Getcb_mN_per_mNm();
	SetTorqueGravity(tq_g_mNm);
	M3Joint::StepCommand();
	

}

}