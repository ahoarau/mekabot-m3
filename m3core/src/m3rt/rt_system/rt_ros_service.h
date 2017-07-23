/* 
M3 -- Meka Robotics Real-Time Control System
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

#ifndef RT_ROS_SERVICE_H
#define RT_ROS_SERVICE_H
#include "m3rt/base/component.h"
#include "m3rt/base/component_base.pb.h"
#include "m3rt/base/toolbox.h"
#include <string>
#include <ros/ros.h>

#ifdef __RTAI__
#ifdef __cplusplus
extern "C" {
#endif 
#include <rtai.h>
#include "rtai_sem.h"
#ifdef __cplusplus
}  // extern "C"
#endif 
#endif


namespace m3rt
{

class M3RtRosService
{
public:
	M3RtRosService(M3RtSystem * s):sys(s),num_clients(0)
	{
		
	}
	bool Startup();					//Called by M3RtService
	void Shutdown();				//Called by M3RtService
	bool AddComponent(std::string name);			//Called by M3RtService
	bool Step();					//Called by M3RtSystem
	void AddClient(){num_clients++;}
	bool AddPublisher(std::string name);
	void RemoveClient(){num_clients--;}
	int GetNumClients(){return num_clients;}
	std::vector<ros::Publisher> publishers;
	std::vector<M3Component *> components_pub;
	//bool RosCallback(m3ros::RosTestMsg::Request  &req, m3ros::RosTestMsg::Response &res);
private:
	//bool RosCallTest(ros::Message & req,ros::Message & res);
	std::vector<M3Component *> components;
	ros::NodeHandle * node_handle_;
	std::vector<ros::ServiceServer> services;
	M3RtSystem * sys;
	int hlt;
	int num_clients;
	
};

}
#endif
