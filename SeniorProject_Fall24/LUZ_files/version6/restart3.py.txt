#!/usr/bin/python3
# coding=utf8

import sys
import os
import time
import signal
import logging
import numpy as np
import pandas as pd

# Add paths for modules
sys.path.append('/home/pi/MasterPi/')
from HiwonderSDK.mecanum import MecanumChassis
from HiwonderSDK.Board import setPWMServoPulse
sys.path.append('/home/pi/MasterPi/HiwonderSDK')
from Sonar import Sonar
sys.path.append('/home/pi/MasterPi/ArmIK')
from ArmIK.ArmMoveIK import ArmIK
sys.path.append ('/home/pi/MasterPi/Functions')
import Avoidance


# Logging configuration
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler("robot.log", mode='w')
    ]
)

class SensorModule:
    """Handles distance measurement using the Sonar sensor."""
    def __init__(self):
        self.sonar = Sonar()
        self.history = []  # Distance history for smoothing

    def get_distance(self):
        """Get smoothed distance from sonar sensor in cm."""
        distance = self.sonar.getDistance()
        if distance is None:
            logging.warning("Sonar sensor failed to detect distance.")
            return 99999  # Assume no object if failure

        # Smooth the distance using a weighted moving average
        distance_cm = distance / 10.0  # Convert mm to cm
        self.history.append(distance_cm)
        if len(self.history) > 5:
            self.history.pop(0)

        smoothed_distance = np.median(self.history)  # Use median for better noise reduction
        logging.info(f"Smoothed distance: {smoothed_distance:.2f} cm")
        return smoothed_distance

class MotionController:
    """Handles the movement of the robot."""
    def __init__(self):
        self.chassis = MecanumChassis()
        self.is_moving = False

    def move_forward(self, velocity=50):
        """Move forward."""
        if not self.is_moving:
            logging.info(f"Moving forward with velocity: {velocity}")
            self.chassis.set_velocity(velocity, 90, 0)
            self.is_moving = True

    def stop(self):
        """Stop all motion."""
        if self.is_moving:
            logging.info("Stopping motion")
            self.chassis.set_velocity(0, 0, 0)
            self.is_moving = False

    def back_up(self, duration=2, velocity=-50):
        """Move backward for a specific duration."""
        logging.info(f"Backing up for {duration} seconds.")
        self.chassis.set_velocity(velocity, 90, 0)
        time.sleep(duration)
        self.stop()

class ArmController:
    """Handles the robotic arm."""
    def __init__(self):
        self.arm = ArmIK()

    def pick_up_object(self):
        """Pick up a detected small object."""
        logging.info("Picking up object...")
        self.arm.setPitchRangeMoving((10, 10, 10), -90, -90, 90, 1500) 
