#!/usr/bin/python
# Antoine Hoarau <hoarau.robotics@gmail.com>

## This allow to check if a running instance exists
## return -1 if already existing
import m3.rt_proxy as m3p
if __name__ == '__main__':
    try:
        proxy = m3p.M3RtProxy()
        proxy.start()
        ## Server is running
        exit(-1)
    except Exception:
        ## Server is NOT running
        exit(0)
