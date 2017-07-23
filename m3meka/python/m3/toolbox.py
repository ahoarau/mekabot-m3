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
#import matplotlib
#matplotlib.use('TkAgg')
import pylab

import os
import sys
import time
import exceptions
import yaml
import Gnuplot
import numpy as nu
#import Numeric
import glob
import datetime
from datetime import timedelta
from m3.unit_conversion import *
from threading import Thread
import m3.component_base_pb2 as mbs

from m3.toolbox_core import *


def get_m3_animation_path():
        try:
                vpath = get_m3_robot_path()
                path  = [p+'/animations/' for p in vpath]
                return path
        except KeyError:
                print 'SET YOUR M3_ROBOT ENVIRONMENT VARIABLE'
        return ''

def get_component_pwr_name(name):
    try:
            with open(get_component_config_filename(name),'r') as f:
                return yaml.safe_load(f.read())['pwr_component']
    except (IOError, EOFError):
            print 'Config file not present for ',name
    return ''

def get_pwr_component_config(name):
    return get_component_config(name)

def get_omnibase_pwr_component_name(name):
    return get_component_pwr_name(name)

def get_chain_pwr_component_name(name):		
        names=get_chain_joint_names(name)
        if len(names)>0:
                return get_joint_pwr_component_name(names[0])
        else:
                return ''

def get_joint_pwr_component_name(name):
    return get_actuator_pwr_component_name(get_joint_actuator_component_name(name))

def get_actuator_pwr_component_name(name):
    return get_actuator_ec_pwr_component_name(get_actuator_ec_component_name(name))

def get_sub_component_name(parent_component_name,sub_component):
    try:
        with open(get_component_config_filename(parent_component_name),'r') as f:
            return yaml.safe_load(f.read())[sub_component]
    except (IOError, EOFError,KeyError) as e:
        print e
    return ''

def get_actuator_ec_pwr_component_name(name):
    return get_sub_component_name(name,'pwr_component')

def get_joint_actuator_component_name(name):	
    if get_component_config_type(name) == 'm3joint_slave':
        return ''
    return get_sub_component_name(name,'actuator_component')
        
def get_joint_ctrl_component_name(name):	
    if get_component_config_type(name) == 'm3joint_slave':
        return ""
    return get_sub_component_name(name,'control_component')


def get_chain_dynamatics_component_name(name):
    return get_sub_component_name(name,'dynamatics_component')

def get_chain_loadx6_name(chain_name):
        if (chain_name.find('arm')==-1):
                print 'Error: get_chain_loadx6_name() valid for arms only'
                return None
        joint_names=get_chain_joint_names(chain_name)
        j0=joint_names[0]
        return 'm3loadx6_'+j0[j0.find('_')+1:j0.find('_',j0.find('_')+1)]+'_l0'

def get_chain_limb_name(name):
    return get_sub_component_name(name,'limb_name')	

def get_chain_joint_names(name):		
        config= get_component_config(name)
        if config.has_key('joint_components'):
                #Have to sort keys by *_JID*'
                ret=[]
                for jid in xrange(len(config['joint_components'])):
                        ret.append(config['joint_components']['J'+str(jid)])
                return ret
        else:
                return []

def get_chain_joint_limits(name):
        l=[]
        names=get_chain_joint_names(name)
        for n in names:
                try:
                        with open(get_component_config_filename(n),'r') as f:
                            config= yaml.safe_load(f.read())
                            l.append([config['param']['min_q'],config['param']['max_q']])
                except (IOError, EOFError):
                        print 'Config file not present:',get_component_config_filename(name),'for',name
                        return []
        return l

def get_actuator_ec_component_name(name):
    return get_sub_component_name(name,'ec_component')

def get_joint_chain_name(joint_name):
        m3_config= get_m3_config()
        comp_type='rt_components'
        try:
            for config in m3_config:
                try: ## new config
                    for components in config[comp_type]: # ma17,mh28..
                        for comp_dir in components:# ma17
                            dict_of_comp={}
                            for comp in components[comp_dir]: #actuator1:type1,actuator2:type2
                                cname = comp.keys()[0]
                                ctype = comp[cname]
                                if ctype in ['m3arm','m3torso' ,'m3hand','m3head']:
                                    dict_of_comp['joint_chain'] = cname
                                dict_of_comp[cname]=ctype
                            if dict_of_comp.get(joint_name,None):
                                try:
                                    return dict_of_comp['joint_chain']
                                except: return ''
                except TypeError:
                    for cdir in config[comp_type]:
                        if joint_name in config[comp_type][cdir]:
                            for c in config[comp_type][cdir]:
                                ctype = config[comp_type][cdir][c]
                                if ctype in ['m3arm','m3torso' ,'m3hand','m3head']:
                                    return c
        except: pass

        return ''

def get_robot_name():
        m3_config= get_m3_config()
        comp_type='rt_components'
        for config in m3_config:
            try:
                for components in config[comp_type]: # ma17,mh28..
                    for comp_dir in components:# ma17
                        dict_of_comp={}
                        for comp in components[comp_dir]: #actuator1:type1,actuator2:type2
                            cname = comp.keys()[0]
                            ctype = comp[cname]
                            if ctype in ['m3humanoid','m3humanoid_sea']:
                                return cname
            except TypeError:
                for cdir in config[comp_type]:
                    for c in config[comp_type][cdir]:
                        ctype = config[comp_type][cdir][c]
                        if ctype in ['m3humanoid','m3humanoid_sea']:
                            return c
        return ''

def __get_hand_name(name):
    config_all= get_m3_config()
    for config in config_all:
        try:
            return config[name]
        except KeyError:
            pass
    return ''


def get_right_hand_name():
    return __get_hand_name('right_hand')

def get_left_hand_name():
    return __get_hand_name('left_hand')

"""
* Animation file reader for files provided by Andrea Thomaz
* each line is a trajectory via point: ts q0 q1 ... qn qd0 qd1 ... qdn qdd0 qdd1 ... qddn
* n dof (7 each arm + torso: n=17) (define joint ordering in the comments)
* ts: timestamp (s)
* qx: joint angle of dof x (rad)
* qdx: joint velocity of dof x (rad/s)
* qddx: joint accel of dof x (rad/s2)

Note: the joint angles seem scale by a factor of 0.5, and the velocities are all >0
       we'll just use the joint angles anyhow
"""

def get_animation_files(extension=['.yml']):
        anim_files=[]
        for ex in extension:
            for p in get_m3_animation_path():
                anim_files.extend(glob.glob(p+'*'+ex))
        return anim_files

def get_animation_file(filename):
    if filename.rfind('.') is -1:
        raise M3Exception("You should add an extension to the filename (you provided "+filename)
    paths=[]
    for p in get_m3_animation_path():
        f = p+filename
        paths.append(f)
        if os.path.isfile(f):
            return f
    print filename,'not found'
    print 'I looked in the following paths:'
    for p in paths:
        print '-',p
    return ''

def get_animation_names():
        names=[]
        for s in get_animation_files(['.txt','.yml']):
                names.append(s[s.rfind('/')+1:s.rfind('.')])
        return names

def load_animation(filename):
        f=open(filename,'rb')
        data=f.read()
        nl=data.find('\r\n')
        samples=[]
        while nl>=0:
                sample=[]
                line=data[:nl]
                ts=line.find('\t')
                while ts>=0:
                        sample.append(float(line[:ts]))
                        line=line[ts+1:]
                        ts=line.find('\t')
                sample.append(float(line))
                samples.append(sample)
                data=data[nl+1:]
                nl=data.find('\r\n')
        x=nu.array(samples)

        #limb names should map to chain limb names
        animation={'timestamp':x[:,0],
                   'torso':{    'theta':x[:,1:4],'thetadot':x[:,18:21],'thetadotdot':x[:,35:38]},
                   'left_arm':{'theta':-2*x[:,4:11], 'thetadot':x[:,21:28],'thetadotdot':x[:,38:45]},
                   'right_arm':{'theta':-2*x[:,11:18], 'thetadot':x[:,28:35],'thetadotdot':x[:,45:52]}}

        ##patch animation to work with meka bodies
        animation['torso']['theta'][:,1:3]=animation['torso']['theta'][:,1:2]*-1
        animation['torso']['thetadot'][:,1:3]=animation['torso']['thetadot'][:,1:2]*-1
        animation['torso']['thetadotdot'][:,1:3]=animation['torso']['thetadotdot'][:,1:2]*-1

        animation['right_arm']['theta'][:,2]=animation['right_arm']['theta'][:,2]*-1
        animation['right_arm']['thetadot'][:,2]=animation['right_arm']['thetadot'][:,2]*-1
        animation['right_arm']['thetadotdot'][:,2]=animation['right_arm']['thetadotdot'][:,2]*-1

        animation['left_arm']['theta'][:,2]=animation['left_arm']['theta'][:,2]*-1
        animation['left_arm']['thetadot'][:,2]=animation['left_arm']['thetadot'][:,2]*-1
        animation['left_arm']['thetadotdot'][:,2]=animation['left_arm']['thetadotdot'][:,2]*-1

        animation['left_arm']['theta'][:,1]=animation['left_arm']['theta'][:,1]*-1
        animation['left_arm']['thetadot'][:,1]=animation['left_arm']['thetadot'][:,1]*-1
        animation['left_arm']['thetadotdot'][:,1]=animation['left_arm']['thetadotdot'][:,1]*-1

        #animation['left_arm']['theta'][:,0]=animation['left_arm']['theta'][:,0]*-1
        #animation['left_arm']['thetadot'][:,0]=animation['left_arm']['thetadot'][:,0]*-1
        #animation['left_arm']['thetadotdot'][:,0]=animation['left_arm']['thetadotdot'][:,0]*-1
        return animation

def get_via_files():
        return get_animation_files(['.via'])

def get_via_names():
        names=[]
        for s in get_via_files():
                names.append(s[s.rfind('/')+1:s.rfind('.')])
        return names