#!/usr/bin/python3
#coding=utf8

import os
import sys
sys.path.append('/home/pi/MasterPi/')
import cv2
import time
import threading
import yaml_handle
import signal
import math
import numpy as np
import argparse
from CameraCalibration.CalibrationConfig import *
import HiwonderSDK.PID as PID
import HiwonderSDK.Sonar as Sonar
import HiwonderSDK.Board as Board
sys.path.append('/home/pi/MasterPi/ArmIK')
import HiwonderSDK.mecanum as mecanum
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *

# Global Constants
Threshold = 80.0  # Distance threshold for obstacle avoidance
size = (640, 480)  # Camera frame size
AK = ArmIK()
HWSONAR = Sonar.Sonar()

# Color Ranges (Only Red, Blue, Green)
range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

# Load color configuration data
lab_data = None
def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

# Define target colors and set detection color
__target_color = ('red', 'green', 'blue')

def setTargetColor(target_color):
    global __target_color
    print("COLOR", target_color)
    __target_color = target_color
    return (True, ())

# Detect Largest Color Area in the Image
def getAreaMaxContour(contours):
    area_temp = 0
    max_area = 0
    area_max_contour = None
    for c in contours:
        area_temp = math.fabs(cv2.contourArea(c))
        if area_temp > max_area and area_temp > 300:  # Ignore small areas
            max_area = area_temp
            area_max_contour = c
    return area_max_contour, max_area  

# Closing angle of the gripper when gripping
servo1 = 1500

# Set distance
x_dis = 1500
y_dis = 860

# PID initializing
x_pid = PID.PID(P=0.28, I=0.03, D=0.03) 
y_pid = PID.PID(P=0.28, I=0.03, D=0.03)
go_pid = PID.PID(P=0.28, I=0.1, D=0.05)
about_pid = PID.PID(P=0.35, I=0.08, D=0.005)

y_speed = 0
x_speed = 0

# Initial position of servo1
def initGrip():
    Board.setPWMServoPulse(1, servo1, 800)
    AK.setPitchRangeMoving((0, 8, 10), 0, -90, 90, 1500)

# Turn off the motor
def MotorStop():
    Board.setMotor(1, 0) 
    Board.setMotor(2, 0)
    Board.setMotor(3, 0)
    Board.setMotor(4, 0)

# Function to set LED color: based on value in r, g, b 
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

count = 0
_stop = False
color_list = []
get_roi = False
__isRunning = False
detect_color = 'None'
start_pick_up = False
start_count_t1 = True

# Variables for tracking()
rect = None
size = (640, 480)

# Variables for move()
rotation_angle = 0
unreachable = False
world_X, world_Y = 0, 0

# Variables for run()
t1 = 0
roi = ()
center_list = []
last_x, last_y = 0, 0
draw_color = range_rgb["black"]

# Variable reset
def reset(): 
    global _stop
    global count
    global get_roi
    global color_list
    global detect_color
    global start_pick_up
    global start_count_t1
    
    # From tracking
    global __isRunning
    global __target_color
    global x_dis, y_dis
    global enableWheel
    x_dis = 1500
    y_dis = 860
    x_pid.clear()
    y_pid.clear()
    go_pid.clear()
    about_pid.clear()
    enableWheel = False
    __target_color = ()
    
    # From run()
    count = 0
    _stop = False
    color_list = []
    get_roi = False
    detect_color = 'None'
    start_pick_up = False
    start_count_t1 = True

def init():
    print("ColorTracking & ColorSorting Init")
    load_config()
    initGrip()
    print(" Load color_config for LAB & gripper at init positoin")
    HWSONAR.setRGBMode(0)
    HWSONAR.setPixelColor(0, Board.PixelColor(0,0,0))
    HWSONAR.setPixelColor(1, Board.PixelColor(0,0,0))    
    HWSONAR.show()

def start():
    global __isRunning
    reset()
    #is reset() needed here?
    __isRunning = True
    print("ColorTracking & ColorSorting Start")
    print(" Start")

def stop():
    global _stop
    global __isRunning
    _stop = True
    reset()
    initGrip()
    MotorStop()
    autonomous_controller.stop()
    __isRunning = False
    set_rgb('None')
    print("ColorTracking Stop")
    print("ColorSorting Stop")

# Set body to follow/tracking() of r,g,b block
#this was orginially false. But i made it true and switched the if else. Not sure if my return is right, though.
#i left the return as it orginaly was.
""" original code
enableWheel = False
def setWheel(Wheel = 0,):
    global enableWheel
    if Wheel :
        enableWheel = True
        Board.setPWMServoPulse(1, 1500, 800)
        AK.setPitchRangeMoving((0, 7, 12), -50, -90, 0, 1500)
    else:
        enableWheel = False
        MotorStop()
        initMove()
    return (True, ())"""

#modified code
enableWheel = True
def setWheel(Wheel=1):
    global enableWheel
    if Wheel:
        enableWheel = False
        MotorStop()
        initGrip()
    else:
        enableWheel = True
        Board.setPWMServoPulse(1, 1500, 800)
        AK.setPitchRangeMoving((0, 7, 12), -50, -90, 0, 1500)
    return (True, ())

# Color Tracking, Color Sorting, Color Detect run () together from source code
#need to thread this function to start the cv2 
def run_move_trace(img):
    global roi
    global rect
    global count
    global _stop
    global get_roi
    global center_list
    global __isRunning
    global enableWheel
    global unreachable
    global start_count_t1, t1
    global detect_color, draw_color, color_list
    global start_pick_up
    global rotation_angle
    global last_x, last_y
    global world_X, world_Y
    global img_h, img_w
    global x_dis, y_dis
    global x_speed, y_speed

    # run() code
    global __target_color
    img_copy = img.copy() 
    img_h, img_w = img.shape[:2]
    
    if not __isRunning:  #make sure its enabled
        return img
    
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # Convert the image to lab space

    max_area = 0
    area_max_contour = 0 
    detected_color = None
    Motor_ = True
    if not start_pick_up:
        for color in lab_data:
            if color in __target_color:
                detected_color = color
                mask = cv2.inRange(frame_lab, 
                                   (lab_data[detected_color]['min'][0], lab_data[detected_color]['min'][1],
                                    lab_data[detected_color]['min'][2]),
                                   (lab_data[detected_color]['max'][0], lab_data[detected_color]['max'][1],
                                    lab_data[detected_color]['max'][2]))
                opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # Open operation
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # Close operation
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                area_max_contour, area_max = getAreaMaxContour(contours)  # Find the maximum contour

                if area_max_contour is not None and area_max > max_area: #find the max area;from colorsorting source code run()
                    max_area = area_max
                    areaMaxCountour_max = area_max_contour
                    cv2.drawContours(img, [box], -1, range_rgb[detected_color], 2)
                    
            #from colordectect run source code        
            if detected_color:
                cv2.putText(img, f"Color: {detected_color}", (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, draw_color, 2)  # Print the detected color on the screen
                set_rgb(detected_color)  # Set the color of the light on the expansion board to the same as the detected color
                
        if max_area > 1000:  # The maximum area has been found; from color Tracking source code
            (center_x, center_y), radius = cv2.minEnclosingCircle(area_max_contour)  # center_x = int(Misc.map(center_x, 0, size[0], 0, img_w))
            center_x = int(Misc.map(center_x, 0, size[0], 0, img_w))
            center_y = int(Misc.map(center_y, 0, size[1], 0, img_h))
            radius = int(Misc.map(radius, 0, size[0], 0, img_w))
            if radius > 100:
                return img
            
            rect = cv2.minAreaRect(area_max_contour)
            box = np.int0(cv2.boxPoints(rect))
                    
            cv2.circle(img, (int(center_x), int(center_y)), int(radius), range_rgb[detect_color], 2)
            
            #tracking() code for blocks
            if enableWheel:
                Motor_ = True
                if abs(center_x - img_w/2.0) < 15:  # The movement is small, so no need to move
                    about_pid.SetPoint = center_x
                else:
                    about_pid.SetPoint = img_w/2.0  # Set
                about_pid.update(center_x)  # Current 
                x_speed = -int(about_pid.output)  # Get PID output value
                x_speed = -100 if x_speed < -100 else x_speed
                x_speed = 100 if x_speed > 100 else x_speed
                
                if abs(center_y - img_h/2.0) < 10:  # The movement is small, so no need to move
                    go_pid.SetPoint = center_y
                else:
                    go_pid.SetPoint = img_h/2.0  
                go_pid.update(center_y)
                y_speed = int(go_pid.output)  # Get PID output value
                y_speed = -100 if y_speed < -100 else y_speed
                y_speed = 100 if y_speed > 100 else y_speed
                
                speed_1 = int(y_speed + x_speed)  # Speed synthesis
                speed_2 = int(y_speed - x_speed)
                speed_3 = int(y_speed - x_speed) 
                speed_4 = int(y_speed + x_speed)

                Board.setMotor(1, speed_1)
                Board.setMotor(2, speed_2)
                Board.setMotor(3, speed_3)
                Board.setMotor(4, speed_4)
            
            else:
                if Motor_:
                    MotorStop()
                    initGrip()
                    Motor_ = False
                    
                    x_pid.SetPoint = img_w / 2.0  # Set 
                    x_pid.update(center_x)  # Current
                    dx = x_pid.output
                    x_dis += int(dx)  # Output
                    x_dis = 500 if x_dis < 500 else x_dis
                    x_dis = 2500 if x_dis > 2500 else x_dis
                    
                    y_pid.SetPoint = img_h / 2.0  # Set
                    y_pid.update(center_y)  # Current 
                    dy = y_pid.output
                    y_dis += int(dy)  # Output 
                    y_dis = 500 if y_dis < 500 else y_dis
                    y_dis = 2500 if y_dis > 2500 else y_dis
                                     
                    Board.setPWMServosPulse([20, 2, 3, int(y_dis), 6, int(x_dis)])
                        
                    #i want the robot to stop when it gets close to the block, and then activate the arm mechanics to pick up the block
                    #and move it out the way. from color detect run() source code
                    #move() code might need to define this fnx on its own...
                    if max_area > 2500:  # The maximum area has been found
                        rect = cv2.minAreaRect(areaMaxCountour_max)
                        box = np.int0(cv2.boxPoints(rect))
                        cv2.drawContours(img, [box], -1, range_rgb[detected_color], 2)
                        if not start_pick_up:
                            if detected_color == 'red':  # Red is the largest
                                color = 1
                            elif detected_color == 'green':  # Green max
                                color = 2
                            elif detected_color == 'blue':  # Blue max
                                color = 3
                            else:
                                color = 0
                            color_list.append(color)
                            if len(color_list) == 3:  # Multiple judgements
                                # Get the average
                                color = int(round(np.mean(np.array(color_list))))
                                color_list = []
                                start_pick_up = True
                                if color == 1:
                                    detect_color = 'red'
                                    draw_color = range_rgb["red"]
                                elif color == 2:
                                    detect_color = 'green'
                                    draw_color = range_rgb["green"]
                                elif color == 3:
                                    detect_color = 'blue'
                                    draw_color = range_rgb["blue"]
                                else:
                                    detect_color = 'None'
                                    draw_color = range_rgb["black"]
                    else:
                        if not start_pick_up:
                            draw_color = (0, 0, 0)
                            detect_color = "None"
                            
                cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)  # Print the detected color on the screen
            return img
        
        
             
        else:
            if Motor_:
                MotorStop()
                initGrip()
                Motor_ = False
    return img, detected_color

def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--Wheel', type=int, default=0, help='0 or 1')
    opt = parser.parse_args()
    return opt

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
#movement when there is no color blocks in its path with normal avoidance based on threshold =80
class AutonomousController(threading.Thread):
    def __init__(self):
        super().__init__()
        self.sensor_mod = SensorMod()
        self.motion_controller = MotionController()
        self.running = True
        print("Initializing SensorMod & MotionController")

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

    def run(self):
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

def Stop(signum, frame):
    global _stop
    global __isRunning
    _stop = True
    autonomous_controller.stop()
    MotorStop()  # Turn off all motors
    __isRunning = False
    cv2.destroyAllWindows()
    reset()
    initGrip()
    set_rgb('None')
    print('Closing...')

# Main Execution (Multithreading)
if __name__ == '__main__':
    opt = parse_opt()
    init()
    start()
    setWheel(**vars(opt))
    __target_color = ("red")
    signal.signal(signal.SIGINT, Stop)
    cap = cv2.VideoCapture('http://127.0.0.1:8080?action=stream')
    
    autonomous_controller = AutonomousController()
    autonomous_controller.start()

    while True:
        ret, img = cap.read()
        if ret:
            frame = img.copy()
            frame, detected_color = run_move_trace(frame)
            
            # Display the processed frame
            frame_resize = cv2.resize(frame, (320, 240))
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    
    cv2.destroyAllWindows()
    stop()