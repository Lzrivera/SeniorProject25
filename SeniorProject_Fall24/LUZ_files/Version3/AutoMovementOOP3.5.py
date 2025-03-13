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
        """
        distance = self.get_distance()
    	if distance is None or distance >= 99999:
        	print("Sonar sensor failed to detect distance. Assuming no object.")
        	return "unknown"
    	distance /= 10.0  # Convert mm to cm
    	if distance < 6.0:  # Small object
        	return "small"
    	elif distance >= 6.0:  # Large object
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
        # Initialize Modules
        self.sensor_module = SensorModule()
        self.motion_controller = MotionController()
        self.arm_controller = ArmController()
        self.running = True
        Avoidance.init()  # Initialize avoidance

    def handle_object(self):
        """
        Determines whether to avoid or pick up an object based on its size.
        - Small (< 6 cm): Pick up and move out of the way
        - Large (>= 6 cm): Avoid with a dynamic movement
        """
        size = self.sensor_module.estimate_object_size()
        if size == "small":
            print("Small object detected. Picking up...")
            self.motion_controller.stop()
            self.arm_controller.pick_up_object()
            self.arm_controller.move_object_out_of_way()
            self.arm_controller.reset_arm()
            print("Object moved. Resuming forward motion...")
        elif size == "large":
            print("Large object detected. Avoiding...")
            self.motion_controller.stop()
            self.motion_controller.turn(angle=90)
            time.sleep(2)
            self.motion_controller.move_forward()
        else:
            print("No object detected. Continuing forward motion...")

    def start(self):
        """
        Main control loop:
        Move forward.
        Continuously detect objects and handle them accordingly.
        """
        try:
            self.motion_controller.move_forward()  # Start moving forward
            while self.running:
                self.handle_object()  # Detect and handle objects
                time.sleep(0.1)  # Short delay between checks
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        """
        Stop all movements and terminate the program.
        """
        self.running = False
        self.motion_controller.stop()
        Avoidance.stop()
        print("Autonomous movement stopped.")

if __name__ == "__main__":
    controller = MainAutonomousController()
    controller.start()
