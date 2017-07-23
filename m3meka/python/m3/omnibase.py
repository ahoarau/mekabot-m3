# -*- coding: utf-8 -*-
#M3 -- Meka Robotics Robot Components
#Copyright (c) 2010 Meka Robotics
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

from m3.vehicle import M3Vehicle
import m3.omnibase_pb2 as mob
from m3.component import M3Component
import yaml
import numpy as nu
import m3.toolbox as m3t
import time
from m3.unit_conversion import *
import math

class M3OmniBase(M3Vehicle):
        """
        The M3OmniBase class has been designed as the principal interface for controlling an M3 omnidirectional mobile base.
        It must be defined in the m3_config.yml file and can be created using the m3.component_factory module.
        The example belows creates an interface for an M3OmniBase defined as M3OmniBase_mb0.
        
        >>> import m3.omnibase as m3o
        >>> omni = m3o.M3OmniBase('M3OmniBase_mb0') # creates M3OmniBase class
        
        The M3OmniBase class can be used to send commands and retrieve status updates to and from the m3 realtime server.  The 
        example below configures the realtime proxy to update the M3OmniBase class with updates from the robot and to recieve commands.
        It also sets the omnibase controller to goal position control, subscribes to the power board to enable motor power,
        and runs the wheel calibration routine if needed.
        
        >>> import m3.rt_proxy as m3p
        >>> proxy = m3p.M3RtProxy()
        >>> proxy.start() # m3rt server must be running first
        >>> proxy.make_operational_all() 
        >>> proxy.subscribe_status(omni)
        >>> proxy.publish_command(omni) 
        >>> proxy.publish_param(omni)
        
        >>> pwr_name=proxy.get_available_components('m3pwr')
        >>> if len(pwr_name)>1:
                pwr_name=m3t.user_select_components_interactive(pwr_name,single=True)
        >>> pwr=m3f.create_component(pwr_name[0])    
        >>> proxy.subscribe_status(pwr)
        >>> proxy.publish_command(pwr) 
        >>> pwr.set_motor_power_on()
        >>> proxy.step()
        
        >>> time.sleep(0.5)    
        >>> omni.calibrate(proxy)
        >>> time.sleep(0.5)
        >>> omni.set_local_position(0,0,0,proxy)
        >>> omni.set_global_position(0,0,0,proxy)
        >>> omni.set_max_linear_accel(0.3)
        >>> omni.set_max_linear_velocity(0.3)
        >>> omni.set_max_rotation_velocity(30)
        >>> omni.set_max_rotation_accel(30)    
        >>> proxy.step()    
        >>> omni.set_mode_traj_goal()
        >>> omni.set_traj_goal(0, 0, 0)    
        >>> proxy.step()
                
        Now the M3OmniBase class can be used to issue global position commands and report our position:
        
        >>> omni.set_traj_goal(2.0, 0, 180)            
        >>> proxy.step()
        >>> print 'Position (x,y,yaw):', omni.get_global_position()
        
       
        """
        
        
        def __init__(self,name):
                M3Vehicle.__init__(self,name,type='m3omnibase')
                self.status=mob.M3OmnibaseStatus()
                self.command=mob.M3OmnibaseCommand()
                self.param=mob.M3OmnibaseParam()
                self.num_casters = 4 

                for i in range(3):                        
                        self.command.opspace_force_desired.append(0)
                        self.command.local_position_desired.append(0)
                        self.command.local_velocity_desired.append(0)
                        self.command.local_acceleration_desired.append(0)
                        self.command.global_position_desired.append(0)
                        self.command.global_velocity_desired.append(0)
                        self.command.global_acceleration_desired.append(0)
                        self.command.traj_goal.append(0)
                        self.command.local_position.append(0)
                        self.command.global_position.append(0)
                for i in range(self.num_casters):
                        self.command.roll_torque_desired.append(0)
                        self.command.steer_torque_desired.append(0)
                        self.command.roll_velocity_desired.append(0)
                        self.command.steer_velocity_desired.append(0)
			self.command.steer_theta_desired.append(0)
                        self.command.caster_mode.append(mob.OMNIBASE_CASTER_OFF)                        
                        self.param.enable_breakbeam.append(0)
		self.vias=[]
		self.via_idx=0
		self.read_config()

        def calibrate(self,proxy):
                """
                Calibrates Omnibase casters if necessary.
                        
                :param proxy: running proxy
                :type proxy: M3RtProxy
                """
                need_to_calibrate = False
                
                for i in range(self.num_casters):                
                        if (not self.is_calibrated(i)):
                                need_to_calibrate = True
                                
                if need_to_calibrate:
			print '------------------------------------------------'
			print 'All casters not calibrated. Do calibration [y]?'
                        if m3t.get_yes_no('y'):
				print 'Note: Orientations are facing robot'
                                print "Turn power on to robot and press any key."
                                raw_input()                                
                                self.set_mode_caster()                                                                                                
                                proxy.step()
                                time.sleep(4)
				caster_names=['FrontRight','RearRight','RearLeft','FrontLeft']
				wiggle = [1,2,1,2]
				last_calib = -1
				repeat_calib = 0
				while need_to_calibrate:					
					for i in [1,2,3,0]:
						if (not self.is_calibrated(i)):
							print '-------------------------------------------'
							print 'Calibrating caster: ', caster_names[i], '..'
							#print 'Manual assist required in CCW direction'
							if i == last_calib:
								repeat_calib += 1
							if repeat_calib == 0:
								wiggle = [1,2,1,2]
								self.home(i,proxy, wiggle[i])
							elif repeat_calib == 1:
								wiggle = [3,0,3,0]
								self.home(i,proxy, wiggle[i])
							elif repeat_calib == 2:
								wiggle = [2,3,0,1]
								self.home(i,proxy, wiggle[i])
							elif repeat_calib >= 3:
								raise m3t.M3Exception('Error calibrating.  Please reposition base and try again.')							
							last_calib = i
							need_to_calibrate = False
							for i in range(self.num_casters):                
								if (not self.is_calibrated(i)):
									need_to_calibrate = True													
                                self.set_mode_caster_off(range(self.num_casters))
                                self.set_mode_off()
                        else:
                                print "Skipping Calibration.."
                
                
        def home(self, idx, proxy, idx_wiggle):
		time_out = 20.0
		caster_names=['FrontRight','RearRight','RearLeft','FrontLeft']
		self.set_mode_caster_off(range(4))
		self.set_mode_caster_theta(idx)						
		self.set_mode_caster_theta(idx_wiggle)	
		self.set_roll_torques(0.0, idx)
                self.enable_breakbeam(idx)                
                start_theta = self.get_steer_theta()[idx]
		#print 'Start theta:', idx, start_theta
		start_theta_wiggle = self.get_steer_theta()[idx_wiggle]
		theta = 0
		theta_cnt = 0
		ts = time.time()
		proxy.step()
		while (not self.is_calibrated(idx)):
			theta_des = start_theta + theta
			self.set_steer_theta(theta_des, idx )
			theta_wig = start_theta_wiggle + 30.0 * math.cos(deg2rad(4.0 * theta_cnt))
			torque_roll = 2.0 * math.cos(deg2rad(6.0 * theta_cnt))
			self.set_steer_theta(theta_wig, idx_wiggle )
			self.set_roll_torques(torque_roll, idx)
			proxy.step()
                        str_tqs = self.get_steer_torques()
			rol_tqs = self.get_roll_torques()
                        print 'Steer Joint Tq at idx', idx, ':', str_tqs[idx]
			print 'Roll Joint Tq at idx', idx, ':', rol_tqs[idx]
			print 'Steer Tq at idx', idx_wiggle, ':', str_tqs[idx_wiggle]
			print '.'                        
			theta_step = 2.0
			theta_cnt += theta_step
			theta_err = theta_des - self.get_steer_theta()[idx]
			print 'theta err:', theta_err
			if theta_err < 40.0:
				theta += theta_step			
			if time.time() - ts > time_out:
				self.disable_breakbeam(idx)
				self.set_mode_caster_off(idx)	
				self.set_mode_caster_off(idx_wiggle)
				self.set_roll_torques(0.0, idx)
				proxy.step()
				return
			time.sleep(0.1)
		self.set_roll_torques(0.0, idx)
		self.set_mode_caster_off(idx)
		self.set_mode_caster_off(idx_wiggle)
		self.disable_breakbeam(idx)                                
		proxy.step()
		print "Caster: ", caster_names[idx], " Calibrated."
		

        def enable_breakbeam(self,idx):
                self.param.enable_breakbeam[idx] = 1
                
        def disable_breakbeam(self,idx):
                self.param.enable_breakbeam[idx] = 0
                
        def is_calibrated(self,idx):
                return self.status.calibrated[idx]
                 
        def set_ctrl_mode(self, mode):
                self.command.ctrl_mode=mode
                
        def set_traj_mode(self, mode):
                self.command.traj_mode=mode
        
        def set_mode_off(self):
                """
                Sets all caster controller modes to off.
                """
                self.command.ctrl_mode=mob.OMNIBASE_CTRL_OFF
               
        def set_mode_cart_local(self):
                self.command.ctrl_mode=mob.OMNIBASE_CTRL_CART_LOCAL

        def set_mode_caster_velocity(self, caster_idx):
                self.command.ctrl_mode=mob.OMNIBASE_CTRL_CASTER
                M3Component.set_int_array(self,self.command.caster_mode,mob.OMNIBASE_CASTER_VELOCITY,caster_idx)
		
	def set_mode_caster_theta(self, caster_idx):
                self.command.ctrl_mode=mob.OMNIBASE_CTRL_CASTER
                M3Component.set_int_array(self,self.command.caster_mode,mob.OMNIBASE_CASTER_THETA,caster_idx)

        def set_mode_caster_torque(self,caster_idx):
                """
                Allows specified caster to be controlled with torque commands and places omnibase in 'caster_mode'.
                        
                :param caster_idx: Index of caster.
                :type caster_idx: array_like, shape < ncasters, optional
                
                :See Also: 	   
                        :meth:`M3OmniBase.set_mode_caster_off`
                        :meth:`M3OmniBase.set_mode_caster`
                """
                self.command.ctrl_mode=mob.OMNIBASE_CTRL_CASTER
                M3Component.set_int_array(self,self.command.caster_mode,mob.OMNIBASE_CASTER_TORQUE,caster_idx)        
                
        def set_mode_caster_off(self,caster_idx):
                """
                Turns off controller for specified caster and places omnibase in 'caster_mode'.
                        
                :param caster_idx: Index of caster.
                :type caster_idx: array_like, shape < ncasters, optional
                
                :See Also: 	   
                        :meth:`M3OmniBase.set_mode_caster_torque`
                        :meth:`M3OmniBase.set_mode_caster`
                """
                self.command.ctrl_mode=mob.OMNIBASE_CTRL_CASTER
                M3Component.set_int_array(self,self.command.caster_mode,mob.OMNIBASE_CASTER_OFF,caster_idx)        
                
        def set_mode_caster(self,mode,caster_idx=None):
                
                M3Component.set_int_array(self,self.command.caster_mode,mode, caster_idx)
        
        def set_mode_traj_goal(self):
                """
                Allows omnibase to be controlled by issuing a goal position in global cartesian space.
                                
                
                :See Also:
                        :meth:`M3OmniBase.is_traj_goal_reached`
                        :meth:`M3OmniBase.set_traj_goal`
                        :meth:`M3OmniBase.set_mode_off`
                        :meth:`M3OmniBase.set_mode_caster`
                """
                self.command.traj_mode = mob.OMNIBASE_TRAJ_GOAL
                self.command.ctrl_mode = mob.OMNIBASE_CTRL_OPSPACE_TRAJ

	def set_mode_traj_via(self):
		self.command.traj_mode = mob.OMNIBASE_TRAJ_VIAS
		self.command.ctrl_mode = mob.OMNIBASE_CTRL_OPSPACE_TRAJ

        def set_mode_joystick(self):
                """
                Allows omnibase to be controlled by joystick commands.
                                
                
                :See Also:
                        :meth:`M3OmniBase.set_joystick_x`
                        :meth:`M3OmniBase.set_joystick_y`
                        :meth:`M3OmniBase.set_joystick_yaw`
                        :meth:`M3OmniBase.set_joystick_button`
                """
                self.command.traj_mode = mob.OMNIBASE_TRAJ_JOYSTICK
                self.command.ctrl_mode = mob.OMNIBASE_CTRL_OPSPACE_TRAJ
                
        def set_mode_caster(self):
                """
                Allows omnibase to be controlled at the caster level as opposed cartestian space.
                
                Additional commands must be issued to set the control mode for each individual caster.
                
                :See Also: 	   
                        :meth:`M3OmniBase.set_mode_caster_torque`
                        :meth:`M3OmniBase.set_mode_caster_off`
                """
                self.command.ctrl_mode=mob.OMNIBASE_CTRL_CASTER

        def set_mode_op_space_force(self):
                self.command.ctrl_mode=mob.OMNIBASE_CTRL_OPSPACE_FORCE
               
        def set_traj_mode_off(self):                
                self.command.traj_mode=mob.OMNIBASE_TRAJ_OFF
               
        def set_local_velocities(self,x_dot,y_dot,heading_dot):
                self.command.local_velocity_desired[0] = x_dot
                self.command.local_velocity_desired[1] = y_dot
                self.command.local_velocity_desired[2] = heading_dot
                
        def set_local_positions(self,x,y,heading):
                self.command.local_position_desired[0] = x
                self.command.local_position_desired[1] = y
                self.command.local_position_desired[2] = heading
                
        def set_local_accelerations(self,x_dotdot,y_dotdot,heading_dotdot):
                self.command.local_acceleration_desired[0] = x_dotdot
                self.command.local_acceleration_desired[1] = y_dotdot
                self.command.local_acceleration_desired[2] = heading_dotdot
        
        def set_roll_torques(self, tq, ind=None):
                """
                Sets roll torque values for selected casters.  A list of caster indexes can be supplied
                to set specific caster torques, or the index
                can be omitted if the length of tq is equal to the number of degrees of casters.
                                
                :param tq: Roll torque values in Nm.
                :type tq: array_like
                :param ind: Index of casters.
                :type ind: array_like, shape(len(tq)), optional
                                
                        
                :See Also: 
                        :meth:`M3OmniBase.set_mode_caster_torque`
                        :meth:`M3OmniBase.set_steer_torques`                        
                """
                M3Component.set_float_array(self,self.command.roll_torque_desired,tq,ind)
                   
        def set_steer_torques(self, tq, ind=None):
                """
                Sets steer torque values for selected casters.  A list of caster indexes can be supplied
                to set specific caster torques, or the index
                can be omitted if the length of tq is equal to the number of degrees of casters.
                                
                :param tq: Steer torque values in Nm.
                :type tq: array_like
                :param ind: Index of casters.
                :type ind: array_like, shape(len(tq)), optional
                                
                        
                :See Also: 
                        :meth:`M3OmniBase.set_mode_caster_torque`
                        :meth:`M3OmniBase.set_roll_torques`                        
                """
                M3Component.set_float_array(self,self.command.steer_torque_desired,tq,ind)
		
	def set_steer_theta(self, th, ind=None):                
                M3Component.set_float_array(self,self.command.steer_theta_desired,th,ind)
                
        def set_steer_velocities(self, v, ind=None):
                M3Component.set_float_array(self,self.command.steer_velocity_desired,v,ind)
	
	def set_roll_velocities(self, v, ind=None):
                M3Component.set_float_array(self,self.command.roll_velocity_desired,v,ind)
		
        def set_max_linear_accel(self, x):
                """
                Sets maximum linear acceleration of omnibase in m/s^2 
                                
                :param x: Max linear acceleration in m/s^2
                :type x: float
                
                .. Note:: Omnibase acceleration is still upper limited by absolute values
                defined by parameters in configuration file.
                        
                :See Also: 
                        :meth:`M3OmniBase.set_max_linear_velocity`
                        :meth:`M3OmniBase.set_max_rotation_accel`
                        :meth:`M3OmniBase.set_max_rotation_velocity`
                """
                self.command.max_linear_acceleration = x
                
        def set_max_linear_velocity(self, x):
                """
                Sets maximum linear velocity of omnibase in m/s
                                
                :param x: Max linear velocity in m/s
                :type x: float
                
                .. Note:: Omnibase velocity is still upper limited by absolute values
                defined by parameters in configuration file.
                        
                :See Also: 
                        :meth:`M3OmniBase.set_max_linear_accel`
                        :meth:`M3OmniBase.set_max_rotation_accel`
                        :meth:`M3OmniBase.set_max_rotation_velocity`
                """
                self.command.max_linear_velocity = x
                
        def set_max_rotation_accel(self, x):
                """
                Sets maximum rotational acceleration of omnibase in deg/sec^2
                                
                :param x: Max rotational acceleration in deg/sec^2
                :type x: float
                
                .. Note:: Omnibase acceleration is still upper limited by absolute values
                defined by parameters in configuration file.
                        
                :See Also: 
                        :meth:`M3OmniBase.set_max_linear_accel`
                        :meth:`M3OmniBase.set_max_linear_velocity`
                        :meth:`M3OmniBase.set_max_rotation_velocity`
                """
                self.command.max_rotation_acceleration = x
                
        def set_max_rotation_velocity(self, x):
                """
                Sets maximum rotational velocity of omnibase in deg/s
                                
                :param x: Max rotational velocity in deg/s
                :type x: float
                
                .. Note:: Omnibase velocity is still upper limited by absolute values
                defined by parameters in configuration file.
                        
                :See Also: 
                        :meth:`M3OmniBase.set_max_linear_accel`
                        :meth:`M3OmniBase.set_max_rotation_accel`
                        :meth:`M3OmniBase.set_max_linear_velocity`
                """
                self.command.max_rotation_velocity = x     

        def set_joystick_x(self, x):
                """
                Sets value of X-axis command from joystick.
                                
                :param x: X-axis joystick command.
                :type x: float (-1.0 <-> 1.0)
                                
                        
                :See Also:
                        :meth:`M3OmniBase.set_mode_joystick`
                        :meth:`M3OmniBase.set_joystick_y`
                        :meth:`M3OmniBase.set_joystick_yaw`
                        :meth:`M3OmniBase.set_joystick_button`
                """
                self.command.joystick_x = x                
                
        def set_joystick_y(self,y):
                """
                Sets value of Y-axis command from joystick.
                                
                :param y: Y-axis joystick command.
                :type y: float (-1.0 <-> 1.0)
                                
                        
                :See Also:
                        :meth:`M3OmniBase.set_mode_joystick`
                        :meth:`M3OmniBase.set_joystick_x`
                        :meth:`M3OmniBase.set_joystick_yaw`
                        :meth:`M3OmniBase.set_joystick_button`
                """
                self.command.joystick_y = y
                
        def set_joystick_yaw(self,yaw):
                """
                Sets value of Yaw-axis command from joystick.
                                
                :param yaw: Yaw-axis joystick command.
                :type yaw: float (-1.0 <-> 1.0)
                                
                        
                :See Also:
                        :meth:`M3OmniBase.set_mode_joystick`
                        :meth:`M3OmniBase.set_joystick_x`
                        :meth:`M3OmniBase.set_joystick_y`
                        :meth:`M3OmniBase.set_joystick_button`
                """
                self.command.joystick_yaw = yaw
                
        def set_joystick_button(self,b):
                """
                Sets value of joystick button command.  Currently a value of -1 should be sent to disable joystick,
                and a value of 0 should be sent to enable joystick in default mode.
                                
                :param b: joystick button command.
                :type b: int [-1,0]
                                
                        
                :See Also:
                        :meth:`M3OmniBase.set_mode_joystick`
                        :meth:`M3OmniBase.set_joystick_x`
                        :meth:`M3OmniBase.set_joystick_y`
                        :meth:`M3OmniBase.set_joystick_yaw`
                """
                self.command.joystick_button = b
        
        def set_op_space_forces(self, x, y, torque):
                self.command.opspace_force_desired[0] = x
                self.command.opspace_force_desired[1] = y
                self.command.opspace_force_desired[2] = torque
                
        def get_global_position(self):
                """
                Gets position of omnibase origin frame in the global frame.
                                
                :returns: position (x,y,yaw) in (m,m,deg)
                :rtype: array, shape (3)
                """
                return nu.array(self.status.global_position,float)

	

        def get_motor_torques(self):
                """
                Gets motor torque values at the actuator level (not joint/caster output).
                                
                :returns: torque values in Nm
                :rtype: array, shape (ncasters*2)
                
                :See Also:
                        :meth:`M3OmniBase.get_steer_torques`
                        :meth:`M3OmniBase.get_roll_torques`                        
                """
                return nu.array(self.status.motor_torque_desired,float)
        
        def get_steer_torques(self):
                """
                Gets steer joint torque values at the caster level.
                                
                :returns: torque values in Nm
                :rtype: array, shape (ncasters)
                
                :See Also:
                        :meth:`M3OmniBase.get_motor_torques`
                        :meth:`M3OmniBase.get_roll_torques`        
                """
                return nu.array(self.status.steer_torque_desired,float)
	
	def get_steer_theta(self):
                """
                Gets steer joint torque values at the caster level.
                                
                :returns: torque values in Nm
                :rtype: array, shape (ncasters)
                
                :See Also:
                        :meth:`M3OmniBase.get_motor_torques`
                        :meth:`M3OmniBase.get_roll_torques`        
                """
                return nu.array(self.status.steer_angle,float)
        	
        
        def get_roll_torques(self):
                """
                Gets roll joint torque values at the caster level.
                                
                :returns: torque values in Nm
                :rtype: array, shape (ncasters)
                
                :See Also:
                        :meth:`M3OmniBase.get_steer_torques`
                        :meth:`M3OmniBase.get_roll_torques`   
                """
                return nu.array(self.status.roll_torque_desired,float)
        
        def get_local_position(self):
                return nu.array(self.status.local_position,float)
        
        def get_desired_position(self):
                return nu.array(self.status.position_desired,float)
                
        def get_desired_acceleration(self):
                return nu.array(self.status.local_acceleration,float)
                
        def get_bus_voltage(self):
                """
                Gets bus voltage for motor power.
                                
                :returns: value in volts
                :rtype: float
                """
                return self.status.bus_voltage
        
        def set_traj_goal(self, x, y, heading):
                """
                Sets desired end location goal in global frame for trajectory controller.
                                
                :param x: desired X-axis value in global frame
                :type x: float
                
                :param y: desired Y-axis value in global frame
                :type y: float
                
                :param heading: desired Yaw-axis value in global frame
                :type heading: float
                        
                :See Also:
                        :meth:`M3OmniBase.set_mode_traj_goal`
                        :meth:`M3OmniBase.is_traj_goal_reached`      
                """
                self.command.traj_goal[0] = x
                self.command.traj_goal[1] = y
                self.command.traj_goal[2] = heading
                
        def is_traj_goal_reached(self):
                """
                Returns true or false depending if the active goal location has been
                reached by the controller.
                                
                :returns: true/false
                :rtype: bool
                
                :See Also:
                        :meth:`M3OmniBase.set_traj_goal`
                        :meth:`M3OmniBase.set_mode_traj_goal`      
                """
                return self.status.traj_goal_reached

        def set_local_position(self,x,y,yaw,proxy):
                """
                Sets the current local position of the odometry system.
                                
                :param x: desired X-axis value in local frame
                :type x: float
                
                :param y: desired Y-axis value in local frame
                :type y: float
                
                :param yaw: desired Yaw-axis value in local frame
                :type yaw: float
                        
                .. Note:: Should be set to zero after starting real-time server component
                because of initial drift caused by non-zero encoder values.
                        
                :See Also:
                        :meth:`M3OmniBase.set_global_position`                        
                """
                self.command.local_position[0] = x
                self.command.local_position[1] = y
                self.command.local_position[2] = yaw
                self.command.adjust_local_position = 1
                proxy.step()
                time.sleep(0.1)
                self.command.adjust_local_position = 0
                proxy.step()
                
        '''def set_local_zero(self):
                self.command.local_position[0] = 0
                self.command.local_position[1] = 0
                self.command.local_position[2] = 0
                self.command.adjust_local_position = 1'''
                
        def set_global_position(self,x,y,yaw,proxy):
                """
                Sets the current global position of the odometry system.
                                
                :param x: desired X-axis value in global frame
                :type x: float
                
                :param y: desired Y-axis value in global frame
                :type y: float
                
                :param yaw: desired Yaw-axis value in global frame
                :type yaw: float
                        
                .. Note:: Should be set to zero after starting real-time server component
                because of initial drift caused by non-zero encoder values.
                        
                :See Also:
                        :meth:`M3OmniBase.set_local_position`                        
                """
                self.command.global_position[0] = x
                self.command.global_position[1] = y
                self.command.global_position[2] = yaw
                self.command.adjust_global_position = 1
                proxy.step()
                time.sleep(0.1)
                self.command.adjust_global_position = 0
                proxy.step()

	def add_via(self,x_des, y_des, yaw_des):
		self.vias.append([[x_des, y_des, yaw_des], 0 , 0])
	
	def load_command(self):
		self.command.ClearField('vias')
		nadd=min(20,len(self.vias)) #only add 20 per cycle to keep packet size down
		for n in range(nadd):
			self.via_idx=self.via_idx+1
			pos_des=self.vias[n][0]
			lin_vel_avg=self.vias[n][1]
			ang_vel_avg=self.vias[n][2]
			self.command.vias.add()
                        for i in range(3):
                                self.command.vias[-1].position_desired.append(pos_des[i])
			self.command.vias[-1].lin_velocity_avg = lin_vel_avg
			self.command.vias[-1].ang_velocity_avg = ang_vel_avg
			self.command.vias[-1].idx=self.via_idx
                        print self.command.vias[-1]
		self.vias=self.vias[nadd:]
                
