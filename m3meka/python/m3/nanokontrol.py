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
#import roslib; roslib.load_manifest('kontrol')
#from kontrol.msg import Kontrol
import time
import rospy
from threading import Thread
from sensor_msgs.msg import Joy
import os
import subprocess
class M3KontrolThread(Thread):
    def __init__(self,verbose=True):
        Thread.__init__(self)
        self.korg  = subprocess.Popen(['rosrun', 'korg_nanokontrol', 'kontrol.py','3'])

        
        self.sliders = [0]*9
        self.knobs = [0]*9
        self.buttons = [0]*18
        self.verbose=verbose

    def start(self):
        if self.verbose:
            print 'Starting M3KontrolThread...'
        rospy.init_node('korg_nanokontrol',anonymous=True,disable_signals=True) #allow Ctrl-C to master process
        rospy.Subscriber("/joy",Joy,self.callback)
        Thread.start(self)
        
    def stop(self): 
        os.system("pkill -P " + str(self.korg.pid))
        os.kill(self.korg.pid,9)
        rospy.signal_shutdown('Exiting')

    def run(self):
        rospy.spin()

    def callback(self,data):
        if self.verbose:
            print data
        self.sliders = data.axes[:len(self.sliders)]
        self.knobs = data.axes[len(self.sliders):len(self.sliders)+len(self.knobs)]
        self.buttons = data.buttons
        if self.verbose:
            print self.sliders
            print self.knobs
            print self.buttons


class M3Kontrol:        
    def __init__(self,verbose=False):
        self.kontrol_thread = M3KontrolThread(verbose)
        
        self.kontrol_thread.start()

    def get_slider(self, idx):
        if idx >= 0 and idx < len(self.kontrol_thread.sliders):
            return self.kontrol_thread.sliders[idx]
        else:
            return 0

    def get_knob(self, idx):
        if idx >= 0 and idx < len(self.kontrol_thread.knobs):
            return self.kontrol_thread.knobs[idx]
        else:
            return 0
    
    def get_button(self, idx):        
        if idx >= 0 and idx < len(self.kontrol_thread.buttons):
            return self.kontrol_thread.buttons[idx]
        else:
            return 0
    
    def stop(self):
        self.kontrol_thread.stop()

    