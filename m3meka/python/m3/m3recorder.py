#!/usr/bin/env python
# Antoine Hoarau <hoarau.robotics@gmail.com>
__version__ = "1.0.0"


import m3
import m3.gui as m3g
import m3.rt_proxy as m3p
import m3.humanoid as m3h
import m3.toolbox as m3t
import argparse
import textwrap

import threading
import time
import m3.head_s2csp_ctrl
import m3.head
import types
import math
import pickle
import numpy as np
from threading import Lock
import os, pwd, sys, time
from collections import defaultdict
import webbrowser


class Clock:
    def tic(self):
        self.start=time.time()
    def tac(self):
        return time.time()-self.start
        
class Recorder(threading.Thread):
    def __init__(self,functions_to_call,args,filename_out,proxy,gui_name='j',folder_path = m3t.get_home_dir(),file_extension = '.txt'):
        threading.Thread.__init__(self)
        if not isinstance(functions_to_call, list):
            functions_to_call = [functions_to_call]
        if not isinstance(args, list):
            args = [args]
        assert type(filename_out) is str
        
        self.folder_path = os.path.abspath(folder_path)+'/'
        assert os.path.isdir(folder_path)
        
        self.gui_name = gui_name
        self.filename_out = filename_out
        
        self.record = True
        self.functions = functions_to_call
        self.proxy=proxy
        self.args = args
        self.file_complete_path=''
        self.file = self.create_recording_file(filename_out, self.folder_path, file_extension)
    
    def create_recording_file(self,filename_out,folder_path,file_extension):
        file_nb = 0
        file_out = None
        file_nb_str = ''
        passed_file_name = False
        while passed_file_name == False:
            try:
                file_name = file_nb_str + filename_out + file_extension
                with open(folder_path + file_name,'r'): 
                    print 'File ' + file_name + ' already exist, incrementing to ' + str(file_nb + 1)
                    file_nb = file_nb + 1
                    if file_nb < 10:
                        file_nb_str = '0' + str(file_nb)+'_'
                    else:
                        file_nb_str = str(file_nb)+'_'
            except IOError:
                break
        print 'Creating a new file : ', file_name
        passed_file_name = True
        self.file_complete_path =  folder_path + file_name
        print 'Path:',self.file_complete_path
        return open(folder_path + file_name, 'w')

    def run(self):
        print self,'Recording...'
        clock = Clock()
        clock.tic()
        n=0
        while self.record:
            j=0
            self.proxy.step()
            for f,arg in zip(self.functions,self.args):
                if arg:                 
                    res = f(arg)
                else:
                    res = f()
               
                if not isinstance(res, list) and not isinstance(res, np.ndarray):
                    res = [res]
                if isinstance(res, np.ndarray):
                    if len(res.shape)==1:
                        self.file.write(' '.join(map(str, res.tolist()))+' ')
                        #np.savetxt(self.file[i], res,fmt='%.5f')
                    else: 
                        res.resize(1,res.shape[0]*res.shape[1])
                        self.file.write(' '.join(map(str, res.tolist()[0]))+' ')
                        ### TO SAVE MAATRIX np.savetxt(self.file, res,fmt='%.5f ')
                if isinstance(res, list):
                    for c in res:
                        self.file.write(str(c)+' ' )
                        
            self.file.write("\n")
            n = n+1

        elapsed = clock.tac()
        if n > 0:
            print self,' recorded ',n,' samples recorded in ',elapsed,'s - frequency : ',(n/elapsed),'Hz'
            print 'File recorded at :',self.file_complete_path
        else:
            print 'No data points recorded for ',self.file_complete_path
        try:
            time.sleep(0.5)
            self.file.close()
        except IOError:
            pass
        print self,'stopped'

    def start_recording(self): 
        self.record=True
    def stop(self):  
        self.record=False  


        

class M3Recorder:
    def __init__(self,bot,enable_zero_gravity,record_now,output_dir,functions_to_call,arguments,proxy=None,filename_out=time.strftime("%Y%m%d-%H%M%S"),add_timestamp=True):
        # Simple arguments checks
        assert type(enable_zero_gravity) is bool
        assert type(record_now) is bool
        assert type(output_dir) is str
        assert os.path.isdir(output_dir)
        assert type(arguments) is list
        assert type(functions_to_call) is list
        assert len(functions_to_call)==len(arguments)
        # The humanoid class
        self.bot = bot
        # Do we add timestamp ?
        self.add_timestamp = add_timestamp
        # Get/create the M3 Proxy to talk with the robot
        if proxy:
            assert isinstance(proxy,m3p.M3RtProxy)
            self.proxy = proxy
        else:
            self.proxy = m3p.M3RtProxy()
        # Start the proxy
        self.proxy.start()
        # Do we send command to the robot ?
        self.enable_zero_gravity=enable_zero_gravity   
        # Bring up the robot
        self.setup_robot(command=enable_zero_gravity)
        # Functions to call as a list of str
        self.functions_to_call=[]
        # Functions to call as a ptr to the function
        self.functions=[]
        # Arguments to pass to the function as list of str
        self.arguments=[]
        # Adding timestamp as first row
        if self.add_timestamp:
            # Adding the str (for printing)
            self.functions_to_call.append('get_timestamp_S')
            # Adding the pointer to the function (for calling)
            self.functions.append(self.get_timestamp_S)
            # Adding the argument (None here)
            self.arguments.append('')
        # Adding the functions passed as arguments
        self.functions_to_call.extend(functions_to_call)
        # Getting the function pointer from the str passed as argument
        # Note: bot could be any class
        for f in functions_to_call:
            self.functions.append(getattr(self.bot,f))
        # Adding the arguments passed as..argument
        self.arguments.extend( arguments )
        # Shall we start recording now ot wait for enter
        self.record_now = record_now
        # The time we started the recording
        self.start_time=0.0
        # If no filename is provided, the name is the date
        if not filename_out:
            filename_out=time.strftime("%Y%m%d-%H%M%S")
        # The generic recorder class
        self.recorder = Recorder(self.functions,self.arguments,filename_out,self.proxy,folder_path=os.path.abspath(output_dir))

    def setup_robot(self,status=True,param=False,command=False,no_shm=True):
        """
            Setup robot to send status/param/cmd via proxy
        """
        self.proxy.subscribe_status(self.bot)
        if param:
            self.proxy.publish_param(self.bot)
        if command:
            self.proxy.make_operational_all()
            if no_shm:
                self.shutdown_shared_memory()
            self.proxy.publish_command(self.bot)
        
    def get_timestamp_S(self):
        """
            Time from start : First row of the recording
        """
        if self.start_time == 0.0:
            self.start_time = self.bot.get_timestamp_uS()*10e-7
        return self.bot.get_timestamp_uS()*10e-7 -self.start_time
        
    def set_to_zero_gravity_mode(self,chain):
        """
            Set robot to floating mode
        """
        self.proxy.step()
        self.bot.set_mode_theta_gc(chain)
        self.bot.set_stiffness(chain, [0.0]*7)
        self.bot.set_theta_deg(chain,self.bot.get_theta_deg(self.chain))
        self.bot.set_slew_rate_proportion(chain,[0.0]*self.ndof)
        self.proxy.step()

    def shutdown_shared_memory(self):
        """
            Place shared memory to SAFEOP if enabled
        """
        if not isinstance(self.bot,m3h.M3Humanoid): return
        humanoid_shm_names=self.proxy.get_available_components('m3humanoid_shm')
        if len(humanoid_shm_names) > 0:
            self.proxy.make_safe_operational(humanoid_shm_names[0])
            
    def start(self):
        print '---------M3 RECORDER--------'
        print 'To be recorded : '
        for f,a in zip(self.functions_to_call,self.arguments):
            print f+'('+a+')'
            if a is not '' and self.enable_zero_gravity:
                self.set_to_zero_gravity_mode(chain=a)

        if not self.record_now:
            print '-------------------------------'
            print 'Press any key to start'
            print 'q to quit'
            c = m3t.get_keystroke()
            if c=='q':
                return None
        self.recorder.start()
        print ""
        print "Press any key to STOP "
        m3t.get_keystroke()

    def stop(self):
        if self.recorder.is_alive():
            self.recorder.stop()
            threading.Thread.join(self.recorder)
        self.proxy.stop()
        
def main(argv):
    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
    description=textwrap.dedent("""
    \t\tWelcome to the M3 Recorder.
    ----------------------------------------------------------------------------
    | This script helps you record stuff on the robot.                         |
    | Example: m3recorder --right_arm get_theta_rad get_thetadot_rad -r        |
    | Author : Antoine Hoarau                                                  |
    ----------------------------------------------------------------------------

    """),epilog='Maintainer: Antoine Hoarau <hoarau.robotics AT gmail DOT com>')

    parser.add_argument('--version', action='version', version='%(prog)s 2.0')
    parser.add_argument('-e','--enable_zero_gravity',action='store_true',help='If True, commands will be sent to the robot to enable zero gravity mode, so be carefull not to send commands to the robot in an other script.')
    parser.add_argument('-r','--record_now',action='store_true',help='Starts recording directly, does not wait for enter to be pressed by user')
    
    working_dir =os.getcwd()
    parser.add_argument('-o','--output_dir',type=str,help='The output dir for your recordings (default is where you launched the script from: '+working_dir+')',default=working_dir)
    parser.add_argument('--filename',type=str,help='Optional filename output')
    
    bot = m3h.M3Humanoid()
    chains = bot.get_available_chains()
    
    import inspect
    no_args_getters = [g[0] for g in inspect.getmembers(m3h.M3Humanoid, predicate=inspect.ismethod) if 'get_' in g[0] and (len(inspect.getargspec( getattr(bot,g[0])).args) == 1)  and not '_M3Humanoid__' in g[0]]
    
    parser.add_argument('-t', type=str,nargs='*',help='Some extra functions',choices=no_args_getters)
    parser.add_argument('-n','--no_time_from_start',help='Remove time from start in s (first row)',action='store_false')
    
    chain_getters = [g[0] for g in inspect.getmembers(m3h.M3Humanoid, predicate=inspect.ismethod) if 'get_' in g[0] and set(inspect.getargspec( getattr(bot,g[0])).args) == set(['self', 'chain'])  and not '_M3Humanoid__' in g[0]]
    
    for c in chains:
        parser.add_argument('--'+c, type=str,nargs='+',help='A list of functions to call for '+c,choices=chain_getters)

    args = parser.parse_args()
    
    arguments = []
    functions = []
    
    try:
        for f in args.t:        
            functions.append(f)
            arguments.append('')
    except: pass

    for c in chains:
        if getattr(args,c):
            for f in getattr(args,c):
                functions.append(f)
                argspec =  inspect.getargspec( getattr(bot,f))
                arguments.append(c)
                
    filename_out = ''
    try: filename_out=args.filename
    except: pass
    
    if not len(functions):
        print 'No functions to record, please see --help more info.'
        exit()
    m3recorder = M3Recorder(bot,args.enable_zero_gravity,args.record_now,args.output_dir,functions,arguments,filename_out=filename_out,add_timestamp=args.no_time_from_start)
    try:
        m3recorder.start()
    except Exception,e: print e
    m3recorder.stop()
if __name__ == '__main__':
    main(sys.argv)
    print 'Exit'
    
