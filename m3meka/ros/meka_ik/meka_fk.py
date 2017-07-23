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


def meka_fk_srv(req):    
    bot.set_theta_sim_rad(req.arm_name, nu.array(req.joint_angles))
    end_pos = bot.get_tool_position_sim(req.arm_name)
    end_rpy = bot.get_tool_roll_pitch_yaw_rad_sim(req.arm_name)
    
    return MekaFKResponse( end_pos, end_rpy)

def meka_fk_server():
    rospy.init_node('meka_fk_node')    
    bot_name=m3t.get_robot_name()    
    global bot
    bot = m3.humanoid.M3Humanoid(bot_name)    
    s = rospy.Service('meka_fk', MekaFK, meka_fk_srv)
    print "Ready to service FK request."
    rospy.spin()

if __name__ == "__main__":
    meka_fk_server()
