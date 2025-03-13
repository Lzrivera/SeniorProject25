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

# Configure logging
logging.basicConfig(
    level=logging.INFO,  # Set the logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler(),  # Log to console
        logging.FileHandler("robot.log", mode='w')  # Log to a file
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
        """
        Estimate the size of the object based on distance variation or other logic.
        """
        distance = self.get_distance()
        if distance is None or distance >= 99999:
            logging.warning("Sonar sensor failed to detect distance. Assuming no object.")
            return "unknown"
        distance /= 10.0  # Convert mm to cm
        if distance < 6.0:  # Small object
            logging.info(f"Object detected: Small ({distance:.1f} cm)")
            return "small"
        elif distance >= 6.0:  # Large object
            logging.info(f"Object detected: Large ({distance:.1f} cm)")
            return "large"
        logging.warning(f"Object size could not be determined: {distance:.1f} cm")
        return "unknown"

class MotionController:
    def __init__(self):
        self.chassis = MecanumChassis()

    def move_forward(self, velocity=50):
        logging.info(f"Moving forward with velocity: {velocity}")
        self.chassis.set_velocity(velocity, 90, 0)

    def turn(self, angle=90, velocity=50):
        direction = "clockwise" if angle > 0 else "counterclockwise"
        logging.info(f"Turning {direction} by {angle} degrees at velocity: {velocity}")
        self.chassis.set_velocity(0, 90, 1 if angle > 0 else -1)
        time.sleep(3)
        self.stop()

    def stop(self):
        logging.info("Stopping motion")
        self.chassis.set_velocity(0, 0, 0)

class ArmController:
    def __init__(self):
        self.arm = ArmIK()

    def pick_up_object(self):
        logging.info("Picking up object...")
        self.arm.setPitchRangeMoving((10, 10, 10), -90, -90, 90, 1500)  # Example position
        setPWMServoPulse(1, 1500, 800)  # Close gripper
        time.sleep(1)

    def move_object_out_of_way(self):
        logging.info("Moving object out of the way...")
        self.arm.setPitchRangeMoving((10, -10, 10), -90, -90, 90, 1500)  # Example move
        time.sleep(1)

    def reset_arm(self):
        logging.info("Resetting arm to default position")
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
        logging.info("Main autonomous controller initialized")

    def handle_object(self):
        """
        Determines whether to avoid or pick up an object based on its size.
        - Small (< 6 cm): Pick up and move out of the way
        - Large (>= 6 cm): Avoid with a dynamic movement
        """
        size = self.sensor_module.estimate_object_size()
        if size == "small":
            logging.info("Handling small object...")
            self.motion_controller.stop()
            self.arm_controller.pick_up_object()
            self.arm_controller.move_object_out_of_way()
            self.arm_controller.reset_arm()
            logging.info("Small object handled successfully")
        elif size == "large":
            logging.info("Handling large object...")
            self.motion_controller.stop()
            self.motion_controller.turn(angle=90)
            time.sleep(2)
            self.motion_controller.move_forward()
        else:
            logging.info("No object detected. Continuing forward motion...")

    def start(self):
        """
        Main control loop:
        Move forward.
        Continuously detect objects and handle them accordingly.
        """
        logging.info("Starting autonomous movement")
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
        logging.info("Stopping autonomous movement")
        self.running = False
        self.motion_controller.stop()
        Avoidance.stop()
        logging.info("Autonomous movement stopped")

if __name__ == "__main__":
    controller = MainAutonomousController()
    controller.start()
