#!/usr/bin/python
'''
Created on Apr 28, 2014

@author: Antoine Hoarau (hoarau.robotics@gmail.com)
'''
import time
import m3.rt_proxy as m3p
import m3.humanoid as m3h
import m3.toolbox as m3t
if __name__ == '__main__':
    proxy = m3p.M3RtProxy()
    proxy.start()
    bot = m3h.M3Humanoid(m3t.get_robot_name())
    bot.initialize(proxy)
    cnt=0
    start = time.time()
    while 1:
        try:
            cnt=cnt+1
            for chain in bot.get_available_chains():
                bot.set_mode_theta_gc(chain)
                bot.set_slew_rate_proportion(chain,[1.0]*bot.get_num_dof(chain))
                bot.set_stiffness(chain, [1.0]*bot.get_num_dof(chain))
                bot.set_theta_deg(chain, [0.0]*bot.get_num_dof(chain)) 
            if(time.time()-start >= 1):
                for chain in bot.get_available_chains():
                    print chain,bot.get_theta_deg(chain)
                print 'freq : ',cnt
                cnt=0
                start = time.time()
            proxy.step()
        except KeyboardInterrupt:
            break
    proxy.stop()
        
