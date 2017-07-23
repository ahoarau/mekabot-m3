#! /usr/bin/python
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
import m3.toolbox_core as m3t
import m3.monitor
import sys
PYTHONPATH = sys.path
if not isinstance(PYTHONPATH,list):
    PYTHONPATH=list(PYTHONPATH)

component_map={
    'm3monitor': m3.monitor.M3Monitor
    }
    
m3_product_codes={}

m3_fpfx={}

m3_eeprom_cfgs={}

config_all = m3t.get_m3_config()
success = False
for config in config_all:
    if 'factory_py_libs' in config:
        success = False
        for k in config['factory_py_libs']:
            if k[0] == '/':  # Full path
                try:
                    execfile(k)
                    print 'M3 INFO: Loading m3 python factory lib at :', k
                    success = True
                    break
                except:
                    pass
            else:  # Look in pythonpath
                for p in PYTHONPATH:
                    if p and not p[-1] == '/':
                        p = p + '/'
                        try:
                            execfile(p + k)
                            print 'M3 INFO: Loading m3 python factory lib at :', p + k
                            success = True
                            break
                        except:
                            pass
                ## and in M3_ROBOT !
                if not success: ## last chance !
                    p = config['config_path']
                    if p and not p[-1] == '/':
                        p = p + '/'
                        try:
                            execfile(p + k)
                            print 'M3 INFO: Loading m3 python factory lib at :', p + k
                            success = True
                            break
                        except:
                            pass
## Generate an error message if could not load the python lib
if not success:
       print("M3 WARNING: Could not load any python factory lib at:")
       for config in config_all:
                if 'factory_py_libs' in config:
		        for k in config['factory_py_libs']:
                                if k[0]=='/':
                                        print k
                                else:
                                        for p in PYTHONPATH:
		                                if not p[-1]=='/':
		                                        p=p+'/'
                                                print '-',p+k
                                        print '-',config['config_path']+k


def create_component(name):
    """This is a useful utility for creating components based
    on the name only. The m3_config.yml file maps component names
    to types. This is used to figure out the type and instantiate
    a new component class"""
    ttype=m3t.get_component_config_type(name)
    if not ttype:
        print 'Component Factory type not found for component',name
        return None
    if not component_map.has_key(ttype):
        print 'Component Factory type ',ttype, 'not found in component_map for',name
        return None
    
    return component_map.get(ttype,None)(name)
