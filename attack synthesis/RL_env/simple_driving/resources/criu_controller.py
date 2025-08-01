
from os import popen, kill
import signal
def get_pid(process_name):
    #ps -ax | grep -v grep | grep 'autotest.py' | awk '{ print $1 }
    pid = popen('ps -ax |  grep -v grep | grep '+process_name+" | awk '{ print $1 }'").read()
    if pid == '':
        return None
    else: 
        return int(pid)


# take checkpoint of running process {pid}
def terminate_criu(process_name):
    pid = get_pid(process_name)
    if pid != None:
            kill(pid, 9)
            return int(pid)
    else:
        return None
   
    
def check_pid(pid):        
    """ Check For the existence of a unix pid. """
    try:
        kill(pid, 0)
    except OSError:
        return False
    else:
        return True
        
    