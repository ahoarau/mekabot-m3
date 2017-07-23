#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Oct 17 15:19:50 2014

@author: Antoine Hoarau
"""
import m3.rt_proxy as m3p
proxy = m3p.M3RtProxy()
import m3.monitor as m
proxy.start()
monitor=m.M3Monitor('m3monitor_m0')
proxy.subscribe_status(monitor)
try:
    while 1:
        proxy.step()
        monitor.pretty_print_status()
        raw_input("Press Enter to refresh (ctrl+c to quit)")
except KeyboardInterrupt:
    pass
proxy.stop()
print 'Exit'