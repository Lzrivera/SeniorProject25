import sys
sys.path.append('/home/pi/MasterPi/')
import time
import signal
import HiwonderSDK.mecanum as mecanum

from ArmIK.InverseKinematics import *
from ArmIK.ArmMoveIK import *

chassis = mecanum.MecanumChassis()

start = True

def Stop(signum, frame):
    global start

    start = False
   
    chassis.set_velocity(0,0,0)  
    

signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':
    AK = ArmIK()
    while start:
        chassis.set_velocity(0,0,0)
        time.sleep(2)
        start = False