#!/usr/bin/python
'''
Created on Apr 28, 2014

@author: Antoine Hoarau (hoarau.robotics@gmail.com)
'''
import time
import m3.rt_proxy as m3p
if __name__ == '__main__':
    proxy = m3p.M3RtProxy()
    proxy.start()
    start = time.time()
    cnt=0
    while 1:
        try:
            cnt=cnt+1
            elapsed = time.time()-start
            if(elapsed >= 1):
                start = time.time()
                print 'cnt:',cnt,'freq : ',cnt
                cnt=0
            proxy.step()
        except KeyboardInterrupt:
            break
    proxy.stop()
        
