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

def ik_test_client():
    rospy.wait_for_service('meka_ik')
    try:
	tmp = MekaIK()
        ik = rospy.ServiceProxy('meka_ik', MekaIK)
        resp1 = ik('right_arm',[0]*3,[0]*3,[0]*7)
        print resp1.success, resp1.angles_solution
        #return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



if __name__ == "__main__":
    ik_test_client()
