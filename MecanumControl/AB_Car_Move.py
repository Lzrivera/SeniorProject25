import sys
sys.path.append('/home/pi/MasterPi/')
import time
import signal
import HiwonderSDK.mecanum as mecanum

chassis = mecanum.MecanumChassis()

start = True

def Stop(signum, frame):
    global start

    start = False
    
    chassis.set_velocity(0,0,0)  
    

signal.signal(signal.SIGINT, Stop)


if __name__ == '__main__':
    while start:
        chassis.set_velocity(50,90,0)
        time.sleep(5)
        """chassis.set_velocity(50,0,0.3)
        time.sleep(2)
        chassis.set_velocity(50,270,0)
        time.sleep(5)
        chassis.set_velocity(50,180,-0.3)
        time.sleep(2)"""
        chassis.set_velocity(50,0,2)
        time.sleep(2)
        chassis.set_velocity(50,180,-2)
        time.sleep(2)
        chassis.set_velocity(0,0,0)
        time.sleep(3)
        #start = False
    Stop(signal.SIGINT, Stop)

        
