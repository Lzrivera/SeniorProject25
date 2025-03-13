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
        chassis.set_velocity(50,90,0)
        time.sleep(5)
        chassis.set_velocity(0,0,0)
        time.sleep(2)
        start = False
# ARM MOVEMENT PORTION OF CODE

    AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90, 1500) 
    time.sleep(1.5) 
    AK.setPitchRangeMoving((5, 6, 18), 0,-90, 90, 1000)
    time.sleep(1.2) 
    AK.setPitchRangeMoving((5, 13, 11), 0,-90, 90, 1000)
    time.sleep(1.2) 
    AK.setPitchRangeMoving((-5, 13, 11), 0,-90, 90, 1000) 
    time.sleep(1.2) 
    AK.setPitchRangeMoving((-5, 6, 18), 0,-90, 90, 1000)  
    time.sleep(1.2) 
    AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90, 1000) 
    time.sleep(1.2) 