#!/usr/bin/python3
# coding=utf8
import sys
import os
sys.path.append('/home/pi/MasterPi/')
import HiwonderSDK.Board as Board
from HiwonderSDK.mecanum import MecanumChassis
from HiwonderSDK.Board import setMotor, setPWMServoPulse
import time
sys.path.append('/home/pi/MasterPi/HiwonderSDK')
from threading import Thread
sys.path.append('/home/pi/MasterPi/')
from Sonar import Sonar
#!/usr/bin/env python3
# encoding:utf-8
import sys
import os
sys.path.append('/home/pi/MasterPi/ArmIK')
from ArmIK.ArmMoveIK import ArmIK
sys.path.append('/home/pi/MasterPi/HiwonderSKD')
from PID import PID
sys.path.append ('/home/pi/MasterPi/Functions')
import Avoidance

class SensorModule:
    def __init__(self):
        self.sonar = Sonar()

    def get_distance(self):
        return self.sonar.getDistance()
    
    def estimate_object_size(self):
        """
        Estimate the size of the object based on distance variation or other logic.
        For this example, we simulate a size estimation for simplicity."""
        
        distance = self.get_distance() / 10.0
        if distance < 6.0:  #small object
            return "small"
        elif distance >= 6.0:  #large object
            return "large"
        return "unknown"

class MotionController:
    def __init__(self):
        self.chassis = MecanumChassis()

    def move_forward(self, velocity=50):
        self.chassis.set_velocity(velocity, 90, 0)

    def turn(self, angle=90, velocity=50):
        if angle > 0:
            self.chassis.set_velocity(0, 90, 1)  # Turn clockwise
        else:
            self.chassis.set_velocity(0, 90, -1)  # Turn counterclockwise
        time.sleep(3)
        self.stop()

    def stop(self):
        self.chassis.set_velocity(0, 0, 0)

    def slant(self, velocity=50, direction=45):
        self.chassis.set_velocity(velocity, direction, 0)

class ArmController:
    def __init__(self):
        self.arm = ArmIK()

    def pick_up_object(self):
        print("Picking up object...")
        self.arm.setPitchRangeMoving((10, 10, 10), -90, -90, 90, 1500)  # Example position
        setPWMServoPulse(1, 1500, 800)  # Close gripper
        time.sleep(1)

    def move_object_out_of_way(self):
        print("Moving object...")
        self.arm.setPitchRangeMoving((10, -10, 10), -90, -90, 90, 1500)  # Example move
        time.sleep(1)

    def reset_arm(self):
        self.arm.setPitchRangeMoving((0, 0, 10), 0, -90, 90, 1500)  # Reset arm
        time.sleep(1)

class MainAutonomousController:
    def __init__(self):
        #Initialize Modules
        self.sensor_module = SensorModule()
        self.motion_controller = MotionController()
        self.arm_controller = ArmController()
        self.running = True
        Avoidance.init() #Initialize avoidance

    def handle_object(self):
        """
        Determines whether to avoid or pick up an object based on its size.
        -small (<6cm: pick up and move out he way
        -Large (>= 6cm: avoid with a dynamic movement
        """
        size = self.sensor_module.estimate_object_size()
        if size == "small" :
            print("Small object detected. Picking up...")
            self.motion_controller.stop()
            self.arm_controller.pick_up_object()
            self.arm_controller.move_object_out_of_way()
            self.arm_controller.reset_arm()
            print("Object moved. Resuming forward motion...")
        elif size == "large" :
            print ("Large object detected. Avoiding...")
            self.motion_controller.stop()
            self.motion_controller.turn(angle=90)
            time.sleep(2)
            self.motion_controller.move_forward()
            #Avoidance.run(None) #use avoidance logic to bypas the object
        else:
            print("No object detected. Continuing foward motion...")
            
    """
   def perform_dynamic_movements(self):
        print("Performing dynamic movements...")
        self.motion_controller.slant(velocity=50, direction=135)  # Example slant movement
        time.sleep(2)
        self.motion_controller.turn(angle=90)  # Example turn movement
        time.sleep(2)
        self.motion_controller.move_forward()  # Resume forward motion
        """
    
    def start(self):
        """
        Main control loop:
        Move forward.
        continuously detect objects and handle them accordinly.
        """
        try:
            self.motion_controller.move_forward() #start moving forward
            while self.running:
                self.handle_object() #updated method call- detect and handle objects
                #self.perform_dynamic_movements()
                time.sleep(.1) #short delay between checks
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        """
        stop all movements and terminate the program"""
        self.running = False
        self.motion_controller.stop()
        Avoidance.stop()
        print("Autonomous movement stopped.")

if __name__ == "__main__":
    controller = MainAutonomousController()
    controller.start()
