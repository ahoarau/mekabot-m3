#! /usr/bin/python
# -*- coding: utf-8 -*-


# M3 -- Meka Robotics Robot Components
# Copyright (C) 2010 Meka Robotics
# Author: edsinger@mekabot.com (Aaron Edsinger)

# M3 is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# M3 is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

# You should have received a copy of the GNU Lesser General Public License
# along with M3.  If not, see <http://www.gnu.org/licenses/>.
import m3
import time
import os
#import roslib; roslib.load_manifest('meka_description')
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import m3.gui as m3g
import m3.rt_proxy as m3p
import m3.arm as m3a
import m3.hand as m3ha
import m3.humanoid as m3h
import m3.omnibase as m3o
import m3.toolbox as m3t
import m3.joint_zlift as m3z
import m3.head_s2csp_ctrl as m3csp
import m3.component_factory as m3f
import time
import math
import numpy as npy
import PyKDL as kdl

def get_zlift(proxy):
    assert isinstance(proxy,m3p.M3RtProxy)
    ## Returns the zlift object, None if none or more is found
    zlift = None
    try:
        zlift = m3z.M3JointZLift('m3joint_mz2_j0')
        proxy.subscribe_status(zlift)
    except Exception,e:
        print 'Zlift not found : ',e
        zlift = None
    return zlift

def get_omnibase(proxy):
    assert isinstance(proxy,m3p.M3RtProxy)
    ## Returns the omnibase object, None if none or more is found
    omni = None
    try:
        omni = m3o.M3OmniBase('m3omnibase_mb2')
        proxy.subscribe_status(omni)
    except Exception,e:
        print 'Omni not found : ',e
        omni = None
    return omni

def get_bot(proxy):
    assert isinstance(proxy,m3p.M3RtProxy)
    ## Returns the bot object, None if none or more is found
    bot = None
    try:
        bot_name = m3t.get_robot_name()
        bot = m3h.M3Humanoid(bot_name)  
        proxy.subscribe_status(bot)
    except Exception,e:
        print 'Something went wrong when trying to get the bot : ',e
        bot = None
    return bot

def get_right_hand(proxy):
    assert isinstance(proxy,m3p.M3RtProxy)
    ## Returns the right hand object, None if none or more is found
    right_hand = None
    try:
        right_hand = m3.hand.M3Hand('m3hand_mh16')   
        proxy.subscribe_status(right_hand)
    except Exception,e:
        print 'No right hand found : ',e
        right_hand = None
    return right_hand

def get_left_hand(proxy):
    assert isinstance(proxy,m3p.M3RtProxy)
    ## Returns the left hand object, None if none or more is found
    left_hand = None
    try:
        left_hand = m3.hand.M3Hand('m3hand_mh28')   
        proxy.subscribe_status(left_hand)
    except Exception,e:
        print 'No left hand found : ',e
        left_hand = None
    return left_hand
def get_head(proxy):
    assert isinstance(proxy,m3p.M3RtProxy)
    csp_rt = None
    try:
        csp_rt=m3.head.M3Head('m3head_ms4')
        proxy.subscribe_status(csp_rt)
    except Exception,e:
        print 'No csp head found : ',e
        csp_rt = None
    return csp_rt
    
def get_right_arm(proxy):
    assert isinstance(proxy,m3p.M3RtProxy)
    right_arm = None
    try:
        right_arm=m3a.M3Arm('m3arm_ma17')
        proxy.subscribe_status(right_arm)
    except Exception,e:
        print 'No right_arm head found : ',e
        right_arm = None
    return right_arm

def get_left_arm(proxy):
    assert isinstance(proxy,m3p.M3RtProxy)
    left_arm = None
    try:
        left_arm=m3a.M3Arm('m3arm_ma20')
        proxy.subscribe_status(left_arm)
    except Exception,e:
        print 'No left_arm head found : ',e
        left_arm = None
    return left_arm

if __name__ == '__main__':
    server_started = False
    while not server_started:
        try:
            proxy = m3p.M3RtProxy()
            proxy.start()
            server_started= True
            
            right_arm = get_right_arm(proxy)
            left_arm  = get_left_arm(proxy)
            head        = get_head(proxy)
            zlift       = get_zlift(proxy)
            omni        = get_omnibase(proxy)
            bot         = get_bot(proxy)
        
            right_hand  = get_right_hand(proxy)
            left_hand   = get_left_hand(proxy)
            if bot:
                all_chains = bot.get_available_chains()
                #right_arm   = 'right_arm' in all_chains
                #left_arm   = 'left_arm' in all_chains
                #head = 'head' in all_chains
                
            print '*************** Available components ***************'
            if zlift:
                print '- Zlift'#,zlift
            if omni:
                print '- Omni'#,omni
            if bot:
                print '- Bot'#,bot
            if right_arm:
                print '- Right Arm'
            if left_arm:
                print '- Left Arm'
            if head:
                print '- Head'#,head
            if right_hand:
                print '- Right hand'#,right_hand
            if left_hand:
                print '- Left hand'#,left_hand
            print '****************************************************'
            
            ## HACK
            ndof_arm = 7
            ndof_hand = 16
            ndof_head = 11
            ndof_zlift = 1
            ndof_base = 3
            # Calibrate ZLift
        #    if zlift is not None:
        #        time.sleep(0.5)
        #        proxy.step()
        #        zlift.calibrate(proxy)
        #        time.sleep(0.5)
        #        proxy.step()
            
            # Calibrate Base
        #    if omni is not None:
        #        time.sleep(0.5)
        #        proxy.step()
        #        omni.calibrate(proxy)
        #        time.sleep(0.5)
        
            proxy.step()
            ## Warm Up
            if omni:
                omni.set_local_position(0,0,0,proxy)
                omni.set_global_position(0,0,0,proxy)
            if zlift:
                calib_zlift = zlift.get_encoder_calibrated()
            ndof_finger = 3
            flex_factor_index = [0.4] * (ndof_finger+1)
            flex_factor_ring = [0.4] * (ndof_finger+1)
            flex_factor_pinky = [0.4] * (ndof_finger+1)
            flex_factor_thumb = [0.4] * (ndof_finger)
            joints = []
            joints.append('X')    
            joints.append('Y')   
            joints.append('yaw')
            joints.append('zlift_joint')
            joints.append('right_arm_j0')
            joints.append('right_arm_j1')
            joints.append('right_arm_j2')
            joints.append('right_arm_j3')
            joints.append('right_arm_j4')
            joints.append('right_arm_j5')
            joints.append('right_arm_j6')
            joints.append('left_arm_j0')
            joints.append('left_arm_j1')
            joints.append('left_arm_j2')
            joints.append('left_arm_j3')
            joints.append('left_arm_j4')
            joints.append('left_arm_j5')
            joints.append('left_arm_j6')
            joints.append('right_hand_j0')
            joints.append('right_hand_j1')
            joints.append('right_hand_j2')
            joints.append('right_hand_j3')
            joints.append('right_hand_j4')
            joints.append('right_hand_j5')
            joints.append('right_hand_j6')
            joints.append('right_hand_j7')
            joints.append('right_hand_j8')
            joints.append('right_hand_j9')
            joints.append('right_hand_j10')
            joints.append('right_hand_j11')
            joints.append('right_hand_j12')
            joints.append('right_hand_j13')
            joints.append('right_hand_j14')
            joints.append('right_hand_j15')
            joints.append('left_hand_j0')
            joints.append('left_hand_j1')
            joints.append('left_hand_j2')
            joints.append('left_hand_j3')
            joints.append('left_hand_j4')
            joints.append('left_hand_j5')
            joints.append('left_hand_j6')
            joints.append('left_hand_j7')
            joints.append('left_hand_j8')
            joints.append('left_hand_j9')
            joints.append('left_hand_j10')
            joints.append('left_hand_j11')
            joints.append('left_hand_j12')
            joints.append('left_hand_j13')
            joints.append('left_hand_j14')
            joints.append('left_hand_j15')
            joints.append('head_j0')
            joints.append('head_j1')
            joints.append('head_j2')
            joints.append('head_j3')
            joints.append('head_j4')
            joints.append('head_j5')
            joints.append('head_j6')
            joints.append('head_j7_rt_eyelid_top')
            joints.append('head_j7_rt_eyelid_bottom')
            joints.append('head_j7_lt_eyelid_top')
            joints.append('head_j7_lt_eyelid_bottom')
               
            rospy.init_node("m3_joint_state_publisher")
            pub = rospy.Publisher("/joint_states", JointState,queue_size=1)
            loop_rate = rospy.Rate(50.0)
            header = Header(0, rospy.Time.now(), '0')
            positions = [0.0]*len(joints)
            print 'Entering ROS Node'
            while not rospy.is_shutdown() and server_started:
                try:
                    header = Header(0, rospy.Time.now(), '0')
                    # Omnibase state
                    proxy.step()
                    i=0
                    if omni:
                        idx = i
                        #omni_torque = omni.get_steer_torques()
                        omni_pos = omni.get_local_position()
                        omni_x = omni_pos[0]
                        omni_y = omni_pos[1]
                        omni_yaw = math.radians(omni_pos[2])
                        positions[idx]=(omni_x); idx=idx+1
                        positions[idx]=(omni_y); idx=idx+1
                        positions[idx]=(omni_yaw); idx=idx+1
                    i=ndof_base
                    
                    idx = i
                    if zlift and calib_zlift:
                        zlift_z = zlift.get_pos_m()
                    else:
                        zlift_z = .5
                    positions[idx]=(zlift_z-(0.32)); #sol->haut_base + haut_base->capteur(repère 0.0)
                        
                    i=i+ndof_zlift
                    if right_arm:
                        idx = i
                        # Arm joint states
                        right_arm_th_rad = right_arm.get_theta_rad()
                        for j in xrange(0,right_arm.get_num_dof()):
                            positions[idx]=(right_arm_th_rad[j]); idx=idx+1
                    i=i+ndof_arm
                    if left_arm:
                        idx = i
                        # Arm joint states
                        left_arm_th_rad = left_arm.get_theta_rad()
                        for j in xrange(0,left_arm.get_num_dof()):
                            positions[idx]=(left_arm_th_rad[j]); idx=idx+1
                    i=i+ndof_arm
                    if right_hand:
                        idx = i
                        # Hand joint states
                        th = right_hand.get_theta_rad()
                        #Thumb
                        positions[idx]=-th[0]+1.57 ; idx=idx+1
                        positions[idx]=th[1] * flex_factor_thumb[0] ; idx=idx+1
                        positions[idx]=th[1] * flex_factor_thumb[1] ; idx=idx+1
                        positions[idx]=th[1] * flex_factor_thumb[2] ; idx=idx+1
                        #Index
                        positions[idx]=th[2] * flex_factor_index[0] ; idx=idx+1
                        positions[idx]=th[2] * flex_factor_index[1] ; idx=idx+1
                        positions[idx]=th[2] * flex_factor_index[2] ; idx=idx+1
                        positions[idx]=th[2] * flex_factor_index[3] ; idx=idx+1
                        #Ring
                        positions[idx]=th[3] * flex_factor_ring[0] ; idx=idx+1
                        positions[idx]=th[3] * flex_factor_ring[1] ; idx=idx+1
                        positions[idx]=th[3] * flex_factor_ring[2] ; idx=idx+1
                        positions[idx]=th[3] * flex_factor_ring[3] ; idx=idx+1
                        #Pinkie
                        positions[idx]=th[4] * flex_factor_pinky[0] ; idx=idx+1
                        positions[idx]=th[4] * flex_factor_pinky[1] ; idx=idx+1
                        positions[idx]=th[4] * flex_factor_pinky[2] ; idx=idx+1
                        positions[idx]=th[4] * flex_factor_pinky[3] ; idx=idx+1
                    i=i+ndof_hand
                    if left_hand:
                        idx = i
                        # Hand joint states
                        th = left_hand.get_theta_rad()
                        #Thumb
                        positions[idx]=-th[0]+1.57 ; idx=idx+1
                        positions[idx]=th[1] * flex_factor_thumb[0] ; idx=idx+1
                        positions[idx]=th[1] * flex_factor_thumb[1] ; idx=idx+1
                        positions[idx]=th[1] * flex_factor_thumb[2] ; idx=idx+1
                        #Index
                        positions[idx]=th[2] * flex_factor_index[0] ; idx=idx+1
                        positions[idx]=th[2] * flex_factor_index[1] ; idx=idx+1
                        positions[idx]=th[2] * flex_factor_index[2] ; idx=idx+1
                        positions[idx]=th[2] * flex_factor_index[3] ; idx=idx+1
                        #Ring
                        positions[idx]=th[3] * flex_factor_ring[0] ; idx=idx+1
                        positions[idx]=th[3] * flex_factor_ring[1] ; idx=idx+1
                        positions[idx]=th[3] * flex_factor_ring[2] ; idx=idx+1
                        positions[idx]=th[3] * flex_factor_ring[3] ; idx=idx+1
                        #Pinkie
                        positions[idx]=th[4] * flex_factor_pinky[0] ; idx=idx+1
                        positions[idx]=th[4] * flex_factor_pinky[1] ; idx=idx+1
                        positions[idx]=th[4] * flex_factor_pinky[2] ; idx=idx+1
                        positions[idx]=th[4] * flex_factor_pinky[3] ; idx=idx+1
                    i=i+ndof_hand
                    if head:
                        idx = i
                        # Head state
                        all_head_joints = head.get_theta_rad()
                        eye_lids_angle_rad = m3t.deg2rad(75.0)
                        for j in xrange(0,len(all_head_joints)-1):
                            positions[idx]=all_head_joints[j] ; idx=idx+1
                        if len(all_head_joints)>0:
                            eye_lids_angle_rad = all_head_joints[-1]
                        for j in xrange(1,5):
                            positions[-j]=eye_lids_angle_rad + m3t.deg2rad(35);
 		    else:
			eye_lids_angle_rad = m3t.deg2rad(75.0)
 			for j in xrange(1,5):
				positions[-j] = eye_lids_angle_rad

                    
                    pub.publish(JointState(header, joints, positions, [], []))
                    loop_rate.sleep()
                except KeyboardInterrupt:
                    break
                except rospy.ROSException:
                    server_started= False
                except Exception,e:
                    print 'Catching exception:',e
                    print 'M3rt serveur seems to be down, waiting for it to reboot : ',e
                    server_started= False
                    time.sleep(1.0)
        except KeyboardInterrupt:
            break
        except rospy.ROSException:
            break
        except Exception,e:
            print 'Catching exception :',e
            print 'Waiting for the M3 server to be launched'
            time.sleep(1.0)
    if proxy:
        proxy.step()
        proxy.stop()
    print 'Exit'
    exit()



