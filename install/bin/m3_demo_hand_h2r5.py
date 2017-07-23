#! /usr/bin/python
# -*- coding: utf-8 -*-

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

import time
import m3.gui as m3g
import m3.humanoid as m3h
import m3.toolbox as m3t
import m3.unit_conversion as m3u
import m3.component_factory as m3f
import numpy as nu
import m3.rt_proxy as m3p
import yaml

class HandDemo:
    def __init__(self,bot,hand_name,gui=None,gui_column=1):
        self.name = hand_name
        self.gui = gui
        self.bot=bot
        self.gui_col = gui_column

        self.data={}
        self.data['param']={}
        self.param = self.data['param']
        self.ndof = self.bot.get_num_dof(self.name)
        try:
            with open(m3t.get_animation_file(self.name+'_postures.yml'),'r') as f:
                self.data= yaml.safe_load(f.read())
                self.param=self.data['param']
                print 'Loaded animation:',self.data
        except:
            print "Animation file not found, using default anim"
            self.param['stiffness']=[1.0]*self.ndof
            self.param['q_slew_rate']=[1.0]*self.ndof
            self.param['grasp_torque']=[200]*self.ndof
            self.data['postures'].append([0.0]*self.ndof)            
            self.data['postures'].append([250.0]*self.ndof)
            
        #Create gui
        self.theta_desire=[0,0,0,0,0]
        self.mode=[1,1,1,1,1]
        self.run=False
        self.run_last=False
        self.running=False
        self.grasp=False
        self.grasp_last=False
        self.grasp_off=False
        self.grasp_off_ts=time.time()
        
        if self.gui:
            self.gui.add('M3GuiTree',   'Param',   (self,'param'),[],[],m3g.M3GuiWrite,column=self.gui_col)
            self.gui.add('M3GuiToggle', 'Animation',      (self,'run'),[],[['Run','Stop']],m3g.M3GuiWrite,self.gui_col)
            self.gui.add('M3GuiModes',  'Joint',      (self,'mode'),range(5),[['Off','Enabled'],1],m3g.M3GuiWrite,self.gui_col)
            self.gui.add('M3GuiSliders','Theta (Deg)', (self,'theta_desire'),range(5),[0,300],m3g.M3GuiWrite,self.gui_col)
            self.gui.add('M3GuiToggle', 'Power Grasp', (self,'grasp'),[],[['Run','Stop']],m3g.M3GuiWrite,self.gui_col)

    def step(self):
        pass

class M3Proc:
    def __init__(self):
        self.proxy = m3p.M3RtProxy()
        self.gui = m3g.M3Gui()

    def stop(self):
        self.proxy.stop()

    def start(self):
        self.proxy.start()
        self.proxy.make_operational_all()

        self.bot = m3h.M3Humanoid()
        self.bot.initialize(self.proxy)
        
        self.chains = [c for c in self.bot.get_available_chains() if 'hand' in c]
        
        self.hands = []
        i=1
        for chain in self.chains:
            self.hands.append(HandDemo(self.bot,chain,self.gui,i)) ; i=i+2

        self.proxy.step()
        self.status_dict=self.proxy.get_status_dict()
        self.gui.add('M3GuiTree',   'Status',    (self,'status_dict'),[],[],m3g.M3GuiRead,2)
        self.gui.start(self.step)

    def step(self):
#        try:
        self.proxy.step()
        self.status_dict=self.proxy.get_status_dict()
        for hand in self.hands:
            self.bot.set_stiffness(hand.name,hand.param['stiffness'])
            self.bot.set_slew_rate_proportion(hand.name,hand.param['q_slew_rate'])
            #Do power Grasp
            if hand.grasp and not hand.grasp_last:
                if hand.mode[0]:
                    self.bot.set_mode_theta_gc(hand.name,[0])
                if hand.mode[1]:
                    self.bot.set_mode_torque_gc(hand.name,[1])
                if hand.mode[2]:
                    self.bot.set_mode_torque_gc(hand.name,[2])
                if hand.mode[3]:
                    self.bot.set_mode_torque_gc(hand.name,[3])
                if hand.mode[4]:
                    self.bot.set_mode_torque_gc(hand.name,[4])
                self.bot.set_theta_deg(hand.name,self.bot.get_theta_deg(hand.name))
                self.bot.set_torque_mNm(hand.name,hand.param['grasp_torque'])
            hand.grasp_last=hand.grasp

            #Do joint theta control
            if not hand.grasp and not hand.running: #theta open
                for jidx in range(5):
                    if hand.mode[jidx]:
                        self.bot.set_mode_theta_gc(hand.name,[jidx])
                    else:
                        self.bot.set_mode_off(hand.name,[jidx])

            #Start Animation
            if not hand.run_last and hand.run and not hand.running:
                hand.running=True
                for jidx in range(5):
                    if hand.mode[jidx]:
                        self.bot.set_mode_theta_gc(hand.name,[jidx])
                    else:
                        self.bot.set_mode_off(hand.name,[jidx])
                hand.pose_idx=0
                hand.ts_anim=time.time()

            if hand.running:
                self.bot.set_theta_deg(hand.name,hand.data['postures'][hand.pose_idx])
                if time.time()-hand.ts_anim>hand.param['pose_time']:
                    hand.ts_anim=time.time()
                    hand.pose_idx=hand.pose_idx+1
                    if hand.pose_idx>=len(hand.data['postures']):
                        hand.running=False
                        print 'Animation done'

            if not hand.running:
                self.bot.set_theta_deg(hand.name,hand.theta_desire)
            hand.run_last=hand.run
#        except Exception,e:
#            print e

if __name__ == '__main__':
    t=M3Proc()
    try:
        t.start()
    except (KeyboardInterrupt,EOFError):
        pass
    t.stop()



