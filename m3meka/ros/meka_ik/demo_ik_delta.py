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
from m3ctrl_msgs.msg import *
from PyKDL import *
from m3.ik_axis import M3IKAxis
import numpy as nu
import m3.joint_mode_ros_pb2 as mab
import m3.smoothing_mode_pb2 as mas
from threading import Thread
import m3.unit_conversion as m3u
import m3.toolbox as m3t
import string

class M3Chain:
    (RIGHT_ARM, LEFT_ARM) = range(2)

def get_pose(arm_name):
    
    cmd = M3JointCmd()
    cmd.chain = [0]*7
    cmd.control_mode = [0]*7
    cmd.smoothing_mode = [0]*7
    
    for i in range(7):
      if arm_name == 'right_arm':
	cmd.chain[i] = (int(M3Chain.RIGHT_ARM))
      elif arm_name == 'left_arm':
	cmd.chain[i] = (int(M3Chain.LEFT_ARM))
      
      cmd.stiffness.append(0.0)
      cmd.position.append(0.0)
      cmd.velocity.append(0.0)
      cmd.effort.append(0.0)
      cmd.control_mode[i] = (int(mab.JOINT_MODE_ROS_TORQUE_GC))
      cmd.smoothing_mode[i] = 0
      cmd.chain_idx.append(i)
    cmd.header = Header(0,rospy.Time.now(),'0')
    humanoid_pub.publish(cmd)
    
    while True:
	print '--------------------------'
	print 'Pose arm'
	print 'Hit enter to set basis pose'
	print '--------------------------'
	os.system("stty raw")
	r = sys.stdin.read(1)
	os.system("stty sane")
	
	return  m3t.deg2rad(nu.array([ 50.0692749,   16.81969261, -17.62414742,  76.4484024,   33.68758011, -28.43738556,  15.0620451]))
	
	'''if r=='\r':
	    if arm_name == 'right_arm':
	      print 'Basis pose of', right_arm_jnt_angles
	      return right_arm_jnt_angles
	    if arm_name == 'left_arm':
	      print 'Basis pose of', left_arm_jnt_angles
	      return left_arm_jnt_angles'''

# ######################################################

class ik_thread(Thread):
    def __init__ (self,step_delta,arm_name):
	Thread.__init__(self)

	self.arm_name = arm_name
	self.verror=0
	self.aerror=0
	self.delta_done=False
	self.delta=nu.zeros(3) 
	self.target_pos=nu.zeros(3)
	
	q = []
	if arm_name == 'right_arm':
	  q = right_arm_jnt_angles
	else:
	  q = left_arm_jnt_angles
	resp_fk = fk_srv(arm_name,q)
	
	self.target_pos_start = resp_fk.end_position
	print 'target_pos_start', self.target_pos_start
	self.target_rpy = resp_fk.end_rpy
	print 'target_rpy', m3t.rad2deg(nu.array(self.target_rpy))
	
	self.step_delta=step_delta
	#self.update=True
	self.update=False
	
    def set_delta(self,x):
	self.delta=nu.array(x)
	self.update=True
	
    def run(self):
	while not self.delta_done:
	    if self.update:
		self.update=False
		self.target_pos=self.target_pos_start+self.delta
		qdes=[]

		qcurrent = []
		if self.arm_name == 'right_arm':
		  qcurrent = right_arm_jnt_angles
		else:
		  qcurrent = left_arm_jnt_angles
		
		resp_ik = ik_srv(self.arm_name,self.target_pos[:],self.target_rpy[:],qcurrent)
		#print resp1.success, resp1.angles_solution
		
		print 'new target_pos ', self.target_pos
		print 'new target_rpy ', m3t.rad2deg(nu.array(self.target_rpy))
		
		if resp_ik.success:
		    cmd = M3JointCmd()
		    cmd.chain = [0]*7
		    cmd.control_mode = [0]*7
		    cmd.smoothing_mode = [0]*7
		    
		    print 'new_q ', m3t.rad2deg(nu.array(resp_ik.angles_solution))
		    
		    for i in range(7):
		      
		      
		      if arm_name == 'right_arm':
			cmd.chain[i] = (int(M3Chain.RIGHT_ARM))
		      elif arm_name == 'left_arm':
			cmd.chain[i] = (int(M3Chain.LEFT_ARM))
		      cmd.stiffness.append(stiffness)
		      cmd.position.append(resp_ik.angles_solution[i])
		      cmd.velocity.append(10.0)
		      cmd.effort.append(0.0)
		      cmd.control_mode[i] = (int(mab.JOINT_MODE_ROS_THETA_GC))
		      cmd.smoothing_mode[i] = (int(mas.SMOOTHING_MODE_SLEW))
		      cmd.chain_idx.append(i)
		    cmd.header = Header(0,rospy.Time.now(),'0')
		    humanoid_pub.publish(cmd)

		q = []
		if arm_name == 'right_arm':
		  q = right_arm_jnt_angles
		else:
		  q = left_arm_jnt_angles
		resp_fk = fk_srv(arm_name,q)
				    
		self.aerror=nu.sqrt(sum((nu.array(resp_fk.end_position)-nu.array(resp_fk.end_rpy))**2))
		
	    time.sleep(0.1)
	    
# ######################################################

def run_ik(step_delta, arm_name):
    pose=get_pose(arm_name)
        
    
    cmd = M3JointCmd()
    cmd.chain = [0]*7
    cmd.control_mode = [0]*7
    cmd.smoothing_mode = [0]*7
    
    for i in range(7):
      if arm_name == 'right_arm':
	cmd.chain[i] = (int(M3Chain.RIGHT_ARM))
      elif arm_name == 'left_arm':
	cmd.chain[i] = (int(M3Chain.LEFT_ARM))
      
      cmd.stiffness.append(stiffness)
      cmd.position.append(pose[i])
      cmd.velocity.append(10.0)
      cmd.effort.append(0.0)
      cmd.control_mode[i] = (int(mab.JOINT_MODE_ROS_THETA_GC))
      cmd.smoothing_mode[i] = (int(mas.SMOOTHING_MODE_SLEW))
      cmd.chain_idx.append(i)
    cmd.header = Header(0,rospy.Time.now(),'0')
    humanoid_pub.publish(cmd)
    
    print 'Press any key to start IK demo.'
    k=m3t.get_keystroke()

    t=ik_thread(step_delta, arm_name)
    t.start()
    d=[0,0,0]
    while 1:
        print '-----------------------'
	print 'Delta: ',t.delta
	print '-----------------------'
	print 'q: quit'
	print '1: x+'
	print '2: x-'
	print '3: y+'
	print '4: y-'
	print '5: z+'
	print '6: z-'
	print 'space: step'
	k=m3t.get_keystroke()
	if k=='q':
	    t.delta_done=True
	    return
	if k=='1':
	    d[0]=d[0]+step_delta
	if k=='2':
	    d[0]=d[0]-step_delta
	if k=='3':
	    d[1]=d[1]+step_delta
	if k=='4':
	    d[1]=d[1]-step_delta
	if k=='5':
	    d[2]=d[2]+step_delta
	if k=='6':
	    d[2]=d[2]-step_delta
	t.set_delta(d)	
	print
	print 'Error: ',t.aerror
	print 'Target: ',t.target_pos

def humanoid_state_callback(data):
    for i in range(len(data.name)):
      j = -1
      if len(data.name[i]) >= 9:
	if string.count(data.name[i],'right_arm') > 0:
	  j = int(data.name[i][-1])
      if j > -1:
	right_arm_jnt_angles[j] = data.position[i]
      j = -1
      if len(data.name[i]) >= 9:
	if string.count(data.name[i],'left_arm') > 0:
	  j = int(data.name[i][-1])
      if j > -1:
	left_arm_jnt_angles[j] = data.position[i]
	  
	
stiffness=0.85
step_delta=.002 #meters

print 'Select arm:'		
arm_names = ['right_arm', 'left_arm']		
arm_name = m3t.user_select_components_interactive(arm_names,single=True)[0]

global right_arm_jnt_angles
global left_arm_jnt_angles

global humanoid_pub
humanoid_pub = rospy.Publisher('humanoid_command', M3JointCmd)

right_arm_jnt_angles = [0.0]*7
left_arm_jnt_angles = [0.0]*7

rospy.init_node('demo_ik', anonymous=True)
rospy.Subscriber("humanoid_state", JointState, humanoid_state_callback)

print 'Waiting for IK service.'
rospy.wait_for_service('meka_ik')
print 'IK service found.'

print 'Waiting for FK service.'
rospy.wait_for_service('meka_fk')
print 'FK service found.'

global ik_srv
ik_srv = rospy.ServiceProxy('meka_ik', MekaIK)

global fk_srv
fk_srv = rospy.ServiceProxy('meka_fk', MekaFK)

'''cmd = M3JointCmd()
cmd.chain = [0]*7
cmd.control_mode = [0]*7
cmd.smoothing_mode = [0]*7

for i in range(7):
  if arm_name == 'right_arm':
    cmd.chain[i] = (int(M3Chain.RIGHT_ARM))
  elif arm_name == 'left_arm':
    cmd.chain[i] = (int(M3Chain.LEFT_ARM))
  
  cmd.stiffness.append(stiffness)
  cmd.position.append(10.0*(3.14/180.0)*i)
  cmd.velocity.append(10.0)
  cmd.effort.append(0.0)
  cmd.control_mode[i] = (int(mab.JOINT_MODE_ROS_THETA_GC))
  cmd.smoothing_mode[i] = (int(mas.SMOOTHING_MODE_SLEW))
  cmd.chain_idx.append(i)
cmd.header = Header(0,rospy.Time.now(),'0')
humanoid_pub.publish(cmd)'''


while True:

    print '--------------'
    print 's: set stiffness (Current',stiffness,')'
    print 'd: set step delta (Current',step_delta,'(m))'
    print 'e: execute ijkt controller'
    print 'q: quit'
    print '--------------'
    print
    k=m3t.get_keystroke()
    if k=='q':
	break
    if k=='s':
	print 'Enter stiffness (0-1.0) [',stiffness,']'
	stiffness=max(0,min(1.0,m3t.get_float(stiffness)))
    if k=='d':
	print 'Enter step delta (m) [',step_delta,']'
	step_delta=max(0,min(.25,m3t.get_float(step_delta)))
    if k=='e':
	run_ik(step_delta,arm_name)


	
    