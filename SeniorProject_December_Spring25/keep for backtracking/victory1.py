import os

#!/usr/bin/python3
#coding=utf8
import sys
import cv2
import time
import signal
sys.path.append('/home/pi/MasterPi')
import Camera
import numpy as np
import pandas as pd
import HiwonderSDK.Sonar as Sonar
#module and class both class sonar this throws up and error; other way to get around it
#from import HiwonderSDK.Sonar import Sonar
import HiwonderSDK.Board as Board
sys.path.append('/home/pi/MasterPi/ArmIK')
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.mecanum as mecanum

#AK = ArmIK()
#chassis = mecanum.MecanumChassis()
##from avoidance script



Threshold = 80.0
#increased threshold limit

class PixelStrip(object):
    def begin(self):
        self.begin = begin()

class SensorMod:
    #Handles distance measurement using Sonar sensor
    def __init__(self):
        self.sonar = Sonar.Sonar()
    
    def getDistance(self):
        #get distance from sonar sensor and print it
        #return self.sonar.getDistance()
        dist =  self.sonar.getDistance() 
        print (dist)
        return dist    

class MotionController:
    #moves chassis forward and stops
    def __init__(self):
        self.chassis = mecanum.MecanumChassis()

    def move(self, velocity, direction, angular_rate=0):
        self.chassis.set_velocity(velocity, direction, angular_rate)
        #move forward in positive velocity, move backward in negative velocity
        #Turn in any direction with 0-360, 

    def stop(self):
        self.chassis.set_velocity(0, 0, 0)
        #stop all motion


class AutonomousController:
    #main control loop for robot
    def __init__(self):
        self.begin()
        self.sensor_mod = SensorMod()
        self.motion_controller = MotionController()
        self.running = True
    
    def object_detected(self):
        dist = self.sensor_mod.getDistance()
        #better to use if and else statements and get return value for this
        if self.running:
            if dist <= Threshold:
                print( "Obstacle detected! Stopping")
                self.motion_controller.stop()
                            
            else:
                if dist >= Threshold:
                    print ("obstacle far away, moving closer")
                    self.motion_controller.move(40, 90)
        return dist
        
            
    def start(self):
        try:
            while self.running:
                self.motion_controller.move(velocity=80, direction=90)
                #moving forward
                time.sleep(5)
                self.object_detected()
                time.sleep(5)
        except KeyboardInterrupt:
            self.stop()
            
    def stop(self):
        self.running = False
        self.motion_controller.stop()
        #self.sensor_mod.close()
        print("autonomous movement stopped")
        
        
if __name__ == '__main__':
    test = AutonomousController()
    test.start()
       
        