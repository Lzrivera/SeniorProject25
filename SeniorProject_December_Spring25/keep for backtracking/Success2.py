import os
import sys

sys.path.append('/home/pi/MasterPi/')
sys.path.append('/home/pi/MasterPi/ArmIK')

import cv2
import time
import threading
import yaml_handle
import signal
import math
import numpy as np
import HiwonderSDK.Sonar as Sonar
import HiwonderSDK.Board as Board
import HiwonderSDK.mecanum as mecanum

#sys.path.append('/home/pi/MasterPi/')
#sys.path.append('/home/pi/MasterPi/ArmIK')

from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *

# Global Constants
Threshold = 80.0  # Distance threshold for obstacle avoidance
size = (640, 480)  # Camera frame size
AK = ArmIK()

# Color Ranges (Only Red, Blue, Green)
range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0)
}

# Sorting Positions for Each Color
sorting_positions = {
    'red': (-15, 14, 2),
    'green': (-18, 9, 3),
    'blue': (-18, 0, 2),
    'capture': (0, 16.5, 2)
}

# Load color configuration data
lab_data = None
def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

# Define target colors
__target_color = ('red', 'green', 'blue')

# Function to set LED color
def set_rgb(color):
    if color in range_rgb:
        r, g, b = range_rgb[color]
        Board.RGB.setPixelColor(0, Board.PixelColor(r, g, b))
        Board.RGB.setPixelColor(1, Board.PixelColor(r, g, b))
        Board.RGB.show()
    else:
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
        Board.RGB.show()

# Detect Largest Color Area in the Image
def getAreaMaxContour(contours):
    max_area = 0
    area_max_contour = None
    for c in contours:
        area = math.fabs(cv2.contourArea(c))
        if area > max_area and area > 300:  # Ignore small areas
            max_area = area
            area_max_contour = c
    return area_max_contour, max_area  

# Color Detection Function
def detect_color(img):
    global __target_color
    img_copy = img.copy()
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

    max_area = 0
    detected_color = None
    for color in __target_color:
        mask = cv2.inRange(frame_lab, 
                           (lab_data[color]['min'][0], lab_data[color]['min'][1], lab_data[color]['min'][2]),
                           (lab_data[color]['max'][0], lab_data[color]['max'][1], lab_data[color]['max'][2]))
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        areaMaxContour, area_max = getAreaMaxContour(contours)

        if areaMaxContour is not None and area_max > max_area:
            max_area = area_max
            detected_color = color

    if detected_color:
        cv2.putText(img, f"Color: {detected_color}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, range_rgb[detected_color], 2)
        set_rgb(detected_color)
    else:
        set_rgb('None')

    return img, detected_color

# Sonar Sensor Class
class SensorMod:
    def __init__(self):
        self.sonar = Sonar.Sonar()

    def getDistance(self):
        return self.sonar.getDistance()

# Motion Control Class
class MotionController:
    def __init__(self):
        self.chassis = mecanum.MecanumChassis()

    def move(self, velocity, direction, angular_rate=0):
        self.chassis.set_velocity(velocity, direction, angular_rate)

    def stop(self):
        self.chassis.set_velocity(0, 0, 0)

# Autonomous Movement Class (Obstacle Avoidance)
class AutonomousController:
    def __init__(self):
        self.sensor_mod = SensorMod()
        self.motion_controller = MotionController()
        self.running = True

    def object_detected(self):
        dist = self.sensor_mod.getDistance()
        if dist <= Threshold:
            print("Object detected! Moving backward")
            self.motion_controller.move(40, 270)
            time.sleep(2)

            dist = self.sensor_mod.getDistance()
            if dist >= Threshold:
                print("Turning")
                self.motion_controller.move(40, 0, 0.3)
                time.sleep(1)
            else:
                print("Obstacle still detected. Moving backward again")
                self.motion_controller.move(40, 270)
                time.sleep(1)

            print("Moving forward after avoiding obstacle")
            self.motion_controller.move(40, 90)
            self.motion_controller.stop()
        else:
            print("No obstacle nearby, moving forward")
            self.motion_controller.move(40, 90)

    def start(self):
        try:
            while self.running:
                self.object_detected()
                time.sleep(0.5)
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        self.running = False
        self.motion_controller.stop()
        print("Autonomous movement stopped")

# Sorting Function
def color_sorting(detected_color):
    if detected_color in sorting_positions:
        target_pos = sorting_positions[detected_color]
        print(f"Sorting {detected_color} to {target_pos}")

        # Move arm to pick up object
        AK.setPitchRangeMoving((0, 6, 18), 0, -90, 90, 1500)
        time.sleep(1.5)
        Board.setPWMServoPulse(1, 2000, 500)  # Open gripper
        time.sleep(1.5)
        Board.setPWMServoPulse(1, 1500, 500)  # Close gripper
        time.sleep(1.5)

        # Move arm to sorting location
        AK.setPitchRangeMoving(target_pos, -90, -90, 0, 800)
        time.sleep(1)

        # Release object
        Board.setPWMServoPulse(1, 1800, 500)
        time.sleep(1)

        # Return arm to starting position
        AK.setPitchRangeMoving((0, 8, 10), -90, -90, 0, 1000)
        time.sleep(1)

# Main Execution (Multithreading)
if __name__ == '__main__':
    load_config()

    # Create instances
    auto_controller = AutonomousController()

    # Start obstacle avoidance in a separate thread
    obstacle_thread = threading.Thread(target=auto_controller.start)
    obstacle_thread.daemon = True
    obstacle_thread.start()

    cap = cv2.VideoCapture('http://127.0.0.1:8080?action=stream')

    while True:
        ret, img = cap.read()
        if ret:
            frame = img.copy()
            processed_frame, detected_color = detect_color(frame)

            frame_resize = cv2.resize(processed_frame, (320, 240))
            cv2.imshow('frame', frame_resize)

            if detected_color:
                sorting_thread = threading.Thread(target=color_sorting, args=(detected_color,))
                sorting_thread.start()

            key = cv2.waitKey(1)
            if key == 27:  # Press 'ESC' to exit
                break
        else:
            time.sleep(0.01)

    cv2.destroyAllWindows()
