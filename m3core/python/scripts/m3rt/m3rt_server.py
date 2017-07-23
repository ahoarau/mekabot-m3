#!/usr/bin/python

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

import time
import xmlrpclib
import SocketServer
import SimpleXMLRPCServer
import sys
import ctypes
from m3.toolbox_core import M3Exception

flags = sys.getdlopenflags()
sys.setdlopenflags(flags | ctypes.RTLD_GLOBAL) #allow exceptions to be passed between dll's
import m3.m3rt_system
import socket
import m3.rt_proxy as m3p
import m3.toolbox_core as m3t
from threading import Thread
from threading import Event
## Handle Ctrl+c even though ros is launched
import signal
stop_signal=Event() 
def stop_program(signal, frame):
    stop_signal.set()

class MyTCPServer(SocketServer.TCPServer):
    def server_bind(self):
        self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.socket.bind(self.server_address)

class MyXMLRPCServer(MyTCPServer, SimpleXMLRPCServer.SimpleXMLRPCDispatcher):
    def __init__(self, addr, requestHandler=SimpleXMLRPCServer.SimpleXMLRPCRequestHandler, logRequests=1):
        self.logRequests = logRequests
        SimpleXMLRPCServer.SimpleXMLRPCDispatcher.__init__(self)
        MyTCPServer.__init__(self, addr, requestHandler)

def start_log_service(logname, freq, components,page_size,logpath=None,verbose=True):
    logdir=m3t.get_log_dir(logname,logpath)
    if logdir is None:
        return False
    if not m3t.make_log_dir(logdir):
        return False
    if len(components)==1 and components[0]=='all':
        components=[]
        n=svc.GetNumComponents()
        for i in range(n):
            components.append(svc.GetComponentName(i))
    if len(components)==0:
        print 'No componentes registered for logging'
        return False
    for c in components:
        svc.AddLogComponent(c)
    return svc.AttachLogService(logname,logdir, freq,page_size,int(verbose)) 

def stop_log_service():
    return svc.RemoveLogService()

def get_log_file(logfilename):
    try:
        with open(logfilename, "rb") as f:
	    s=f.read()
            return xmlrpclib.Binary(s)
    except IOError:
        return ''

def get_log_info(logname,logpath=None):
    return m3t.get_log_info(logname,logpath)


class client_thread(Thread):
    def __init__ (self, make_all_op = False, make_all_op_shm = False, make_all_op_no_shm = True,data_svc=False):
        Thread.__init__(self)
        self.make_all_op = make_all_op
        self.make_all_op_shm = make_all_op_shm
        self.make_all_op_no_shm = make_all_op_no_shm
        self.stop_event = Event()
        self.proxy = m3p.M3RtProxy(rpc_port=port)
        self.proxy.start(start_data_svc, False)  
        print "M3 INFO: M3 is now running ",
        if self.make_all_op:
            print "(with option -make operational all+shm)"
            self.proxy.make_operational_all()
            self.proxy.make_operational_all_shm()
        if self.make_all_op_shm:  
            print "(with option -make operational shm only)"
            self.proxy.make_operational_all_shm()        
        if self.make_all_op_no_shm:
            print "(with option -make operational all (no shm))"
            self.proxy.make_operational_all()
    def run(self):
        self.stop_event.wait(timeout=None)
        print "M3 INFO: Closing Client Thread."
        self.proxy.stop()    

class M3Server(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.server = MyXMLRPCServer((host,port),logRequests=False)
        self.server.register_introspection_functions()
        self.server.register_instance(svc)
        self.server.register_function(start_log_service)
        self.server.register_function(stop_log_service)
        self.server.register_function(get_log_file)
        self.server.register_function(get_log_info)
        #time.sleep(2.0) # wait for EC kmod to get slaves in OP
    def run(self):
        print 'Starting M3 RPC Server on Host:',host,'at Port:',port,'...'
        self.server.serve_forever()
        print "M3 INFO: Closing Socket"
        self.server.socket.close()
            
# ################################################################################

## THE DEFAULT ARGUMENTS TO START THE SERVER
host = m3t.get_local_hostname()
port=8000
make_op_all = False
make_op_all_shm = False
make_op_all_no_shm = True
start_data_svc = False

svc = None
m3server = None
m3client_thread = None
signal.signal(signal.SIGINT, stop_program)

for idx in range(1,len(sys.argv)):
    if sys.argv[idx]=='-host' or sys.argv[idx]=='-h' and idx<len(sys.argv)-1:
        host=sys.argv[idx+1]
    elif sys.argv[idx]=='-port' or sys.argv[idx]=='-p' and idx<len(sys.argv)-1:
        port=int(sys.argv[idx+1])
    elif sys.argv[idx]=='-make_op_all' or sys.argv[idx]=='-m':
        make_op_all = True
	make_op_all_no_shm = False
    elif sys.argv[idx]=='-make_op_all_shm' or sys.argv[idx]=='-s':
        make_op_all_shm = True
	make_op_all_no_shm = False
    elif sys.argv[idx]=='-make_op_all_no_shm' or sys.argv[idx]=='-n':
        make_op_all_no_shm = True
    elif sys.argv[idx]=='-start_data_svc' or sys.argv[idx]=='-d':
        start_data_svc = True
    elif idx == 1 or sys.argv[idx]=='-help' or sys.argv[idx]=='--help':
        print ''
        print 'M3RT valid arguments:'
        print '   -h, -host <hostname>   specify hostname for server'
        print '   -p, -port <port>       specify port number for server'
        print '   -m, -make_op_all       launch server and place all components in mode Operational'
        print '   -s, -make_op_all_shm       launch server and place shared memory components in mode Operational'
        print '   -n, -make_op_all_no_shm       launch server and place all components except shared memory in mode Operational'
        print '   -d, -start_data_svc    start data service'
        print '   -help                  this help screen'
        print ''
        sys.exit()

try:
    svc=m3.m3rt_system.M3RtService()
    if not svc.Startup(): # Let client start rt_system
	print("M3 ERROR: M3RtService failed to start, exiting.")
	exit(-1)
    #for i in xrange(40):
    #    time.sleep(0.05)
    # Instantiate the server
    while not svc.IsServiceThreadActive():
        time.sleep(0.05)
    try:
        m3server = M3Server()
    except Exception,e:
        print "M3 ERROR: Error creating the server:",e
        raise M3Exception("M3 RPC Server failed to start")
    
    # Start the server
    #print "M3 INFO: Starting M3Rt."
    m3server.start()
    #print "M3 INFO: Starting client thread."
    try:
        m3client_thread = client_thread(make_op_all , make_op_all_shm , make_op_all_no_shm,start_data_svc)
        #for i in xrange(400):
         #   time.sleep(0.01)
    except Exception,e:
        print "M3 ERROR: Error creating the client thread:",e
        raise M3Exception("Client Thread failed to start")
    
    m3client_thread.start()
    #print "M3 INFO: M3 is now running."
    # Handling ctrl+c when ros is launched
    while svc.IsRtSystemOperational() and not stop_signal.is_set():
        try:
			time.sleep(0.250)
        except KeyboardInterrupt:
            print 'M3 INFO: Shutdown signal caught.'
    print "M3 INFO: Shutdown initiated."
except Exception,e:
    print 'M3 ERROR:',e  
    
if svc:
    print "M3 INFO: Shutting down M3Service."
    svc.Shutdown() #it's in the destructor !
 
if m3client_thread and m3client_thread.is_alive():
    print "M3 INFO: Shutting down Client Thread."
    m3client_thread.stop_event.set()
    print 'M3 INFO: Waiting for client thread to shutdown.'
    while m3client_thread.is_alive():
        time.sleep(0.05)
    print 'M3 INFO: Client thread exited normally.'
  
if m3server and m3server.is_alive():
    print "M3 INFO: Shutting down M3 RPC Server."
    m3server.server.shutdown()
    while m3server.is_alive():
	print 'M3 INFO: Waiting for M3 RPC Server to shutdown.'
        time.sleep(0.05)
    print 'M3 INFO: M3 RPC Server exited normally.'

time.sleep(0.5)
print("M3 INFO: Exiting")
exit(0)
# ################################################################################
    
    
    
