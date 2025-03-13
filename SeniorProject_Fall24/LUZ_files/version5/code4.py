#!/usr/bin/python3
# coding=utf8

import sys
import os
import time
import signal
import logging

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
        self.history = []  # Keep a history of distances for smoothing

    def get_distance(self):
        """Get smoothed distance from sonar sensor in cm."""
        distance = self.sonar.getDistance()
        if distance is None:
            logging.warning("Sonar sensor failed to detect distance.")
            return 99999  # Assume no object if failure
        
        # Smooth the distance using a moving average
        distance_cm = distance / 10.0  # Convert mm to cm
        self.history.append(distance_cm)
        if len(self.history) > 5:
            self.history.pop(0)
        
        smoothed_distance = sum(self.history) / len(self.history)
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
        self.arm.setPitchRangeMoving((10, 10, 10), -90, -90, 90, 1500)  # Example position
        setPWMServoPulse(1, 1500, 800)  # Close gripper
        time.sleep(1)

    def move_object_out_of_way(self):
        """Move the object out of the way."""
        logging.info("Moving object out of the way...")
        self.arm.setPitchRangeMoving((10, -10, 10), -90, -90, 90, 1500)
        time.sleep(1)

    def reset_arm(self):
        """Reset the arm to the default position."""
        logging.info("Resetting arm to default position.")
        self.arm.setPitchRangeMoving((0, 0, 10), 0, -90, 90, 1500)
        time.sleep(1)

class MainAutonomousController:
    """Main control loop for the robot."""
    def __init__(self):
        self.sensor_module = SensorModule()
        self.motion_controller = MotionController()
        self.arm_controller = ArmController()
        self.running = True
        Avoidance.init()

    def handle_small_object(self):
        """Handle a small object."""
        logging.info("Small object detected. Handling it...")
        self.motion_controller.stop()
        self.arm_controller.pick_up_object()
        self.arm_controller.move_object_out_of_way()
        self.arm_controller.reset_arm()
        self.motion_controller.move_forward()

    def handle_large_object(self):
        """Handle a large object using avoidance."""
        logging.info("Large object detected. Initiating avoidance.")
        Avoidance.start()
        while True:
            distance = Avoidance.run()  # Use Avoidance logic
            if distance > 30.0:  # Break loop if obstacle is cleared
                break
        Avoidance.stop()
        self.motion_controller.move_forward()

    def handle_object(self):
        """Handle detected objects based on their size and distance."""
        distance = self.sensor_module.get_distance()
        logging.info(f"Detected object at distance: {distance:.2f} cm")

        if 2.0 <= distance <= 10.0:  # Small object threshold
            self.handle_small_object()
        elif 10.1 <= distance <= 30.0:  # Large object threshold
            self.handle_large_object()
        elif distance > 30.0 and distance <= 45.72:  # Far object within a foot
            logging.info("Object above 30 cm within a foot. Avoiding...")
            self.handle_large_object()
        else:
            # Only call move_forward if not already moving
            if not self.motion_controller.is_moving:
                self.motion_controller.move_forward()

    def start(self):
        """Start the main control loop."""
        logging.info("Starting autonomous movement.")
        try:
            self.motion_controller.move_forward()
            while self.running:
                self.handle_object()
                time.sleep(0.2)  # Slightly longer delay for smoother motion
        except KeyboardInterrupt:
            self.stop()
        except Exception as e:
            logging.error(f"Unexpected error: {e}")
            self.stop()

    def stop(self):
        """Stop the robot."""
        logging.info("Stopping autonomous movement.")
        self.running = False
        self.motion_controller.stop()
        Avoidance.stop()
        logging.info("Autonomous movement stopped.")

if __name__ == "__main__":
    controller = MainAutonomousController()
    controller.start()
