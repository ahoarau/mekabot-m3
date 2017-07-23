#! /usr/bin/python
# -*- coding: utf-8 -*-


#M3 -- Meka Robotics Robot Components
#Copyright (C) 2010 Meka Robotics
#Author: edsinger@mekabot.com (Aaron Edsinger)

#M3 is free software: you can redistribute it and/or modify
#it under the terms of the GNU Lesser General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.

#M3 is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU Lesser General Public License for more details.

#You should have received a copy of the GNU Lesser General Public License
#along with M3.  If not, see <http://www.gnu.org/licenses/>.

import time
import os
import roslib; roslib.load_manifest('meka_ik')
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from meka_ik.srv import *
from PyKDL import *
from m3.ik_axis import M3IKAxis
import numpy as nu
import m3.humanoid
import m3.toolbox as m3t

#global bot

def joint_update(joint_states):
    for i in range(len(joints)):
      for j in range(len(joint_states.name)):
	if joints[i] == joint_states.name[j]:
	  positions[i] = joint_states.position[j]
	  velocities[i] = joint_states.velocity[j]
	  effort[i] = joint_states.effort[j]

'''rospy.init_node("meka_ik")
pub = rospy.Publisher("/humanoid_command", JointState)

rospy.Subscriber("/humanoid_state", JointState, joint_update)
'''
joints = []
positions = []


joints.append('right_arm_j0')
positions.append(0.0)
joints.append('right_arm_j1')
positions.append(0.0)
joints.append('right_arm_j2')
positions.append(0.0)
joints.append('right_arm_j3')
positions.append(0.0)
joints.append('right_arm_j4')
positions.append(0.0)
joints.append('right_arm_j5')
positions.append(0.0)
joints.append('right_arm_j6')
positions.append(0.0)


def meka_ik_srv(req):    
    qdes=[]
    #success = self.bot.get_tool_position_rpy_2_theta_deg(arm_name, self.target_pos[:], self.target_rpy[:], qdes)
    bot.set_theta_sim_rad(req.arm_name, nu.array(req.angles_current))
    success = bot.get_tool_position_rpy_2_theta_rad_sim(req.arm_name, req.end_position[:], req.end_rpy[:], qdes)

    if not success:
      qdes = [0.0]*7
    return MekaIKResponse(success, qdes[:])

def meka_ik_server():
    rospy.init_node('meka_ik_node')    
    bot_name=m3t.get_robot_name()    
    global bot
    bot = m3.humanoid.M3Humanoid(bot_name)    
    s = rospy.Service('meka_ik', MekaIK, meka_ik_srv)
    print "Ready to service IK request."
    rospy.spin()

if __name__ == "__main__":
    meka_ik_server()
