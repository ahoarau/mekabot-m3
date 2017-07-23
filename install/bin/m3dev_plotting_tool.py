#! /usr/bin/python
'''
Created on Nov 22, 2013

@author: Antoine Hoarau
'''
import m3.gui as m3g
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.unit_conversion as m3u
import m3.component_factory as m3f
import numpy as nu
import m3.humanoid as m3h
import m3.toolbox_head_s2 as m3th
import m3.head_s2csp_ctrl as m3csp
import math
import random
import time

import Gnuplot
import matplotlib

if __name__=='__main__':
    th_plot = m3t.M3ScopeN(n=7,title='Theta')
    thdot_plot = m3t.M3ScopeN(n=7,title='ThetaDot')
    thdotdot_plot = m3t.M3ScopeN(n=7,title='ThetaDotDot')
    pos_plot = m3t.M3ScopeN(n=3,title='pos xyz')
    vel_plot = m3t.M3ScopeN(n=3,title='vel cart')
    acc_plot = m3t.M3ScopeN(n=3,title='acc cart')
    tq_plot = m3t.M3ScopeN(n=7,title='torque')
    tqgrav_plot = m3t.M3ScopeN(n=7,title='torque gravity')
    f_plot = m3t.M3ScopeN(n=3)
    proxy = m3p.M3RtProxy()
    proxy.start()
    bot_name = m3t.get_robot_name()
    bot = m3h.M3Humanoid(bot_name)
    proxy.subscribe_status(bot)
    
    #scope.plot(np.concatenate((bot.get_tool_velocity('right_arm'),xdot_d[:3])))
    tstart = time.time()
    n = 0
    print 'tstart  : ',tstart
    end = False
    chain = 'right_arm'
    while not end:
        try:
            #ts = time.time()
            proxy.step()
            th = bot.get_theta_deg(chain)
            th_plot.plot(th)
            thdot = bot.get_thetadot_deg(chain)
            thdot_plot.plot(thdot)
            thdotdot = bot.get_thetadotdot_deg(chain)
            tq = bot.get_torque(chain)
            tq_plot.plot(tq)
            tqgrav = bot.get_torque_gravity(chain)
            tqgrav_plot.plot(tqgrav)
            #print 'step : ',time.time()-ts
            n = n + 1
            
        except KeyboardInterrupt:
            th_plot.stop()
            thdot_plot.stop()
            thdotdot_plot.stop()
            elapsed = time.time()-tstart
            frequency  = n/elapsed
            print 'Elapsed time : ',elapsed
            print 'n = ',n
            print 'Frequency : ',frequency
            proxy.stop()
            end = True
            print 'exit'
    exit()
    
    
    
