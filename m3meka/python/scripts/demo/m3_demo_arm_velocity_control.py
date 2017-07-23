#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 30 20:55:08 2014

@author: Antoine Hoarau
"""

"""
This is just a simple demo of the new velocity_gc mode, setting vel to zero"
"""

import m3.rt_proxy as m3p
import m3.humanoid as m3h

proxy = m3p.M3RtProxy()
proxy.start()

bot = m3h.M3Humanoid()
bot.initialize(proxy)
proxy.step()

import m3.toolbox as m3t
arm = m3t.user_select_components_interactive(['right_arm','left_arm'],single=True)[0]

bot.set_mode_thetadot_gc(arm)
bot.set_slew_rate_proportion(arm,[1.0]*bot.get_num_dof(arm))
bot.set_stiffness(arm,[1.0]*bot.get_num_dof(arm))
bot.set_thetadot_deg(arm,[0.0]*bot.get_num_dof(arm))
print("This will set the velocity to 0 (tries to stay still)")
raw_input("Press Enter to start")
proxy.step()
raw_input("Press Enter to stop")
proxy.stop()
exit()
