#!/usr/bin/python3
# coding=utf8
import sys
import os
sys.path.append('/home/pi/MasterPi/')
import HiwonderSDK.Board as Board
from ArmIK.InverseKinematics import *
from ArmIK.ArmMoveIK import *
import signal
import HiwonderSDK.mecanum as mecanum
from HiwonderSDK.mecanum import MecanumChassis
from HiwonderSDK.Board import setMotor, setPWMServoPulse
import time
sys.path.append('/home/pi/MasterPi/HiwonderSDK')
from threading import Thread
sys.path.append('/home/pi/MasterPi/')
from Sonar import Sonar
#!/usr/bin/env python3
# encoding:utf-8
import logging
sys.path.append('/home/pi/MasterPi/ArmIK')
from ArmIK.ArmMoveIK import ArmIK
sys.path.append('/home/pi/MasterPi/HiwonderSKD')
from PID import PID
sys.path.append ('/home/pi/MasterPi/Functions')
import Avoidance

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler("robot.log", mode='w')
    ]
)

class SensorModule:
    def __init__(self):
        self.sonar = Sonar()

    def get_distance(self):
        distance = self.sonar.getDistance()
        logging.debug(f"Raw distance reading: {distance} mm")
        return distance

    def estimate_object_size(self):
        distance = self.get_distance()
        if distance is None or distance >= 99999:
            logging.warning("Sonar sensor failed to detect distance. Assuming no object.")
            return "unknown"
        distance /= 10.0  # Convert mm to cm
        logging.info(f"Measured distance: {distance:.1f} cm")
        if distance < 6.0:  # Small object
            return "small"
        elif distance >= 6.0:  # Large object
            return "large"
        return "unknown"

class MotionController:
    def __init__(self):
        self.chassis = MecanumChassis()

    def move_forward(self, velocity=50):
        logging.info(f"Moving forward with velocity: {velocity}")
        self.chassis.set_velocity(velocity, 90, 0)  # Forward with no rotation

    def stop(self):
        logging.info("Stopping motion")
        self.chassis.set_velocity(0, 0, 0)

class ArmController:
    def __init__(self):
        self.arm = ArmIK()

    def pick_up_object(self):
        logging.info("Picking up object...")
        self.arm.setPitchRangeMoving((10, 10, 10), -90, -90, 90, 1500)
        setPWMServoPulse(1, 1500, 800)  # Close gripper
        time.sleep(1)

    def move_object_out_of_way(self):
        logging.info("Moving object out of the way...")
        self.arm.setPitchRangeMoving((10, -10, 10), -90, -90, 90, 1500)
        time.sleep(1)

    def reset_arm(self):
        logging.info("Resetting arm to default position")
        self.arm.setPitchRangeMoving((0, 0, 10), 0, -90, 90, 1500)
        time.sleep(1)

class MainAutonomousController:
    def __init__(self):
        self.sensor_module = SensorModule()
        self.motion_controller = MotionController()
        self.arm_controller = ArmController()
        self.running = True
        Avoidance.init()

    def handle_object(self):
        size = self.sensor_module.estimate_object_size()
        logging.info(f"Detected object size: {size}")
        if size == "small":
            logging.info("Small object detected. Picking up...")
            self.motion_controller.stop()
            self.arm_controller.pick_up_object()
            self.arm_controller.move_object_out_of_way()
            self.arm_controller.reset_arm()
            logging.info("Object moved. Resuming forward motion...")
            self.motion_controller.move_forward()
        elif size == "large":
            logging.info("Large object detected. Starting avoidance routine...")
            self.motion_controller.stop()
            Avoidance.start()  # Start avoidance logic
            Avoidance.run(None)  # Pass None as the image parameter
            Avoidance.stop()  # Stop avoidance once done
            logging.info("Avoidance complete. Resuming forward motion...")
            self.motion_controller.move_forward()
        else:
            logging.info("No object detected. Continuing forward motion...")
            self.motion_controller.move_forward()

    def start(self):
        logging.info("Starting autonomous movement")
        try:
            self.motion_controller.move_forward()  # Start moving forward
            while self.running:
                self.handle_object()  # Detect and handle objects
                time.sleep(0.1)  # Short delay between checks
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        logging.info("Stopping autonomous movement")
        self.running = False
        self.motion_controller.stop()
        Avoidance.stop()
        logging.info("Autonomous movement stopped")

if __name__ == "__main__":
    controller = MainAutonomousController()
    controller.start()
