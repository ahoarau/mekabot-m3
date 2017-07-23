#!/usr/bin/python
# Antoine Hoarau <hoarau.robotics@gmail.com>

## This allow to check if a running instance exists
## return -1 if already existing
import os
import subprocess
sub = subprocess.Popen(['python','/home/hoarau/dev/mekabot/install/bin/m3rt_check_server_popen.py'],stdout = subprocess.PIPE)
ret = sub.wait()
exit(ret)
