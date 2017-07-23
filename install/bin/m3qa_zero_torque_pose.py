#! /usr/bin/python

#Copyright  2008, Meka Robotics
#All rights reserved.
#http://mekabot.com

#Redistribution and use in source and binary forms, with or without
#modification, are permitted. 


#THIS SOFTWARE IS PROVIDED BY THE Copyright HOLDERS AND CONTRIBUTORS
#"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#Copyright OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES INCLUDING,
#BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#POSSIBILITY OF SUCH DAMAGE.
import pylab as pyl
import time
import m3.toolbox as m3t
import m3.rt_proxy as m3p
import m3.component_factory as mcf
import m3.humanoid 
import m3.hand
import m3.gui as m3g
import numpy as nu
import yaml
import numpy.numarray as na
import math
from m3qa.calibrate_sensors import *

class M3Proc:
	def __init__(self):
		self.proxy = m3p.M3RtProxy()
		self.gui = m3g.M3Gui(stride_ms=50)
		
		
		
	def stop(self):
		self.proxy.stop()

	def start(self):
		# ######## Setup Proxy and Components #########################
		self.proxy.start()
		self.current_first = True
		

		
		bot_name=m3t.get_robot_name()
		if bot_name == "":
			print 'Error: no robot components found:', bot_names
			return
		self.bot=m3.humanoid.M3Humanoid(bot_name)	
		arm_names = self.bot.get_available_chains()	
		arm_names = [x for x in arm_names if x.find('arm')!=-1]
		if len(arm_names)==0:
			print 'No arms found'
			return
		if len(arm_names)==1:
			self.arm_name=arm_names[0]
		else:
			self.arm_name = m3t.user_select_components_interactive(arm_names,single=True)[0]
			
		self.jnts = self.bot.get_joint_names(self.arm_name)
	    	
	    	comp={}
	    	
	    	for c in self.jnts:
		    comp[c]={'comp_rt':None,'comp_j':None,'torque_act':[],'torque_joint':[],'torque_gravity':[],'is_wrist':False}
		    if (c.find('j5')>=0 or c.find('j6')>=0):
			    comp[c]['is_wrist']=True
			    
	    	for c in self.jnts:		  
		    comp[c]['comp_j']=mcf.create_component(c)
		    comp[c]['comp_rt']=mcf.create_component(c.replace('joint','actuator'))
		    self.proxy.subscribe_status(comp[c]['comp_rt'])
		    self.proxy.subscribe_status(comp[c]['comp_j'])

		# ####### Setup Proxy #############
		self.proxy.subscribe_status(self.bot)
		self.proxy.publish_command(self.bot)
		self.proxy.make_operational_all()
		self.bot.set_motor_power_on()
		self.ndof=self.bot.get_num_dof(self.arm_name)
		
		humanoid_shm_names=self.proxy.get_available_components('m3humanoid_shm')
		if len(humanoid_shm_names) > 0:
		  self.proxy.make_safe_operational(humanoid_shm_names[0])
		  
		self.bot.set_mode_off(self.arm_name)

		print 'This script will calibrate the zero-torque of the A2 arm while posed using current torque_gravity estimate'
		print '----------------------------------------------------------------------------------------------------------'
		print 'With E-Stop down, pose the arm in desired position to calibrate torque zeroes around.'
		print 'Press any key when ready posed.'
		
		raw_input()   
		
		time.sleep(0.5)
		
		self.proxy.step()

		self.theta_curr = self.bot.get_theta_deg(self.arm_name)[:]			
		
		self.proxy.step()

		print 'Posed position set.'
		print 'Release E-stop and press any key for arm to hold pose.'
		
		raw_input()   
		
		self.bot.set_mode_theta_gc(self.arm_name)
		self.bot.set_theta_deg(self.arm_name,self.theta_curr)
		self.bot.set_stiffness(self.arm_name,[1.0]*7)
		self.bot.set_slew_rate_proportion(self.arm_name,[1.0]*self.ndof)
		
		self.proxy.step()

		print 'Press any key to start torque calibration for all joints.'		
		
		raw_input()   
		
		self.proxy.step()

		# ###########################
		ns=30
		for i in range(ns):
			self.proxy.step()
			print '---------'
			for c in comp.keys():
				tqj=comp[c]['comp_j'].get_torque_mNm()
				tqg=comp[c]['comp_j'].get_torque_gravity_mNm()/1000.0
				tqa=comp[c]['comp_rt'].get_torque_mNm()
				comp[c]['torque_act'].append(tqa)
				comp[c]['torque_joint'].append(tqj)
				comp[c]['torque_gravity'].append(tqg)
				if comp[c]['is_wrist']:
					print c,':joint',tqj,':gravity',tqg,':actuator',tqa
				else:
					print c,':joint',tqj,':gravity',tqg,
			time.sleep(0.05)

		do_query = True
			
		# ###########################
		for c in comp.keys():
			print '--------',c,'---------'
			tqg=float(na.array(comp[c]['torque_gravity'],na.Float32).mean())
			tqj=float(na.array(comp[c]['torque_joint'],na.Float32).mean())
			tqa=float(na.array(comp[c]['torque_act'],na.Float32).mean())
			if not comp[c]['is_wrist']:
				bias=tqa+tqg
				torque=M3TorqueSensor(comp[c]['comp_rt'].config['calib']['torque']['type'])
				print 'Measured torque:',tqa,'Torque gravity:', tqg
				print 'Delta of',bias,'mNm'
				comp[c]['comp_rt'].config['calib']['torque']['cb_bias']=comp[c]['comp_rt'].config['calib']['torque']['cb_bias']-bias
				comp[c]['comp_rt'].config['calibration_date']=time.asctime()
				if do_query:
					print 'Save calibration? [y]'
					if m3t.get_yes_no('y'):
						comp[c]['comp_rt'].write_config()
				else:
					comp[c]['comp_rt'].write_config()
			else: 
				print 'Wrist joint...'
				if c.find('j5')!=-1: #do j5/j6 at once
					cc=None
					for x in comp.keys():
						if x.find('j6')!=-1:
							cc=x
					if cc is None:
						print 'Did not find coupled joint to',c
					tqg_c=float(na.array(comp[cc]['torque_gravity'],na.Float32).mean())
					tqj_c=float(na.array(comp[cc]['torque_joint'],na.Float32).mean())
					tqa_c=float(na.array(comp[cc]['torque_act'],na.Float32).mean())
					x=comp[c]['comp_j'].config['transmission']['tqj_to_tqa'][0] #Joint to actuator matrix
					y=comp[c]['comp_j'].config['transmission']['tqj_to_tqa'][1]
					m=comp[cc]['comp_j'].config['transmission']['tqj_to_tqa'][0]
					n=comp[cc]['comp_j'].config['transmission']['tqj_to_tqa'][1]
					tqg_a5= x*tqg+y*tqg_c
					tqg_a6= m*tqg_c+n*tqg
					bias_5=tqa+tqg_a5
					bias_6=tqa_c+tqg_a6
					torque_5=M3TorqueSensor(comp[c]['comp_rt'].config['calib']['torque']['type'])
					torque_6=M3TorqueSensor(comp[cc]['comp_rt'].config['calib']['torque']['type'])
					print '------------'
					print 'J5: Previous joint torque',tqj,'with joint torque gravity', tqg
					print 'J5: Previous actuator torque',tqa,'with actuator torque gravity', tqg_a5
					print 'J5: Actuator delta of',bias_5,'mNm'
					print '------------'
					print 'J6: Previous joint torque',tqj_c,'with joint torque gravity', tqg_c
					print 'J6: Previous actuator torque',tqa_c,'with actuator torque gravity', tqg_a6
					print 'J6: Actuator delta of',bias_6,'mNm'
					print '------------'
					comp[c]['comp_rt'].config['calib']['torque']['cb_bias']=comp[c]['comp_rt'].config['calib']['torque']['cb_bias']-bias_5
					comp[c]['comp_rt'].config['calibration_date']=time.asctime()
					comp[cc]['comp_rt'].config['calib']['torque']['cb_bias']=comp[cc]['comp_rt'].config['calib']['torque']['cb_bias']-bias_6
					comp[cc]['comp_rt'].config['calibration_date']=time.asctime()
					if do_query:
						print 'Save calibration? [y]'
						if m3t.get_yes_no('y'):
							comp[c]['comp_rt'].write_config()
							comp[cc]['comp_rt'].write_config()
					else:
						comp[c]['comp_rt'].write_config()
						comp[cc]['comp_rt'].write_config()		
		
		self.bot.set_mode_off(self.arm_name)
		self.proxy.stop() 



if __name__ == '__main__':
	t=M3Proc()
	try:
		t.start()
	except (KeyboardInterrupt,EOFError):
		pass
	t.stop()





