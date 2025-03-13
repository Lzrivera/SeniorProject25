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

# Add paths for external libraries
#sys.path.append('/home/pi/MasterPi/')
#sys.path.append('/home/pi/MasterPi/ArmIK')

from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *

# Global Constants
Threshold = 80.0  # Distance threshold for obstacle avoidance
size = (640, 480)  # Camera frame size
AK = ArmIK()

# Define color ranges for detection (ONLY Red, Blue, Green)
range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0)
}

# Load color configuration data
lab_data = None
def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

# Define target colors (ONLY Red, Blue, Green)
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

# Function to detect the largest color area in the image
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
def run(img):
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

    return img

# Sonar Sensor Class
class SensorMod:
    def __init__(self):
        self.sonar = Sonar.Sonar()

    def getDistance(self):
        dist = self.sonar.getDistance()
        print(f"Distance: {dist} cm")
        return dist

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
        print("Autonomous Controller Initialized")
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

# Main Execution (Multithreading)
if __name__ == '__main__':
    load_config()  # Load color detection configuration

    # Create instances
    auto_controller = AutonomousController()

    # Start obstacle avoidance in a separate thread
    obstacle_thread = threading.Thread(target=auto_controller.start)
    obstacle_thread.daemon = True
    obstacle_thread.start()

    # Start video capture for color detection
    cap = cv2.VideoCapture('http://127.0.0.1:8080?action=stream')

    # Process frames for color detection
    while True:
        ret, img = cap.read()
        if ret:
            frame = img.copy()
            processed_frame = run(frame)

            frame_resize = cv2.resize(processed_frame, (320, 240))
            cv2.imshow('frame', frame_resize)

            key = cv2.waitKey(1)
            if key == 27:  # Press 'ESC' to exit
                break
        else:
            time.sleep(0.01)

    # Cleanup
    cv2.destroyAllWindows()

