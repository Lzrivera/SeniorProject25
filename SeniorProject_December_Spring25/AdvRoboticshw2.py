#!/usr/bin/python3
# coding=utf8
# import os
import sys

sys.path.append('/home/pi/MasterPi/')
import Camera
import argparse

import cv2
import time
import signal
# import Camera
# import argparse
import threading
import yaml_handle

import math
import numpy as np

import HiwonderSDK.PID as PID  # used in ColorTracking
import HiwonderSDK.Misc as Misc

import HiwonderSDK.Sonar as Sonar
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

sys.path.append('/home/pi/MasterPi/ArmIK')

import HiwonderSDK.mecanum as mecanum  # used in Autonomous_controller
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *

# Global Constants
Threshold = 80.0  # Distance threshold for obstacle avoidance
# size = (640, 480)  # Camera frame size
AK = ArmIK()  # the robot arm moves according to the angle calculated
# by inverse kinematics
HWSONAR = Sonar.Sonar()  # ultrasonic sensor

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
    global lab_data, servo_data

    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)


# Define target colors and set detection color
__target_color = ('blue',)  # to track one specific color.  treated as a single-element tuple


def setTargetColor(target_color):
    global __target_color

    print("COLOR", target_color)
    __target_color = target_color
    return (True, ())  # data returns true, and () is an empty placeholder for a tuple


# Detect Largest Color Area in the Image
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    areaMaxContour = None

    for c in contours:  # traverse all contours
        contour_area_temp = math.fabs(cv2.contourArea(c))  # calc contour area
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:  # the contour with the largest area is
                # valid only when the area is greater than 300 to filer out interference
                areaMaxContour = c
    return areaMaxContour, contour_area_max  # largest contour


# closing angle of the gripper when gripping
servo1 = 1500

# set distance
x_dis = 1500
y_dis = 860

# pid initializing
x_pid = PID.PID(P=0.28, I=0.03, D=0.03)
y_pid = PID.PID(P=0.28, I=0.03, D=0.03)

go_pid = PID.PID(P=0.28, I=0.1, D=0.05)
about_pid = PID.PID(P=0.35, I=0.08, D=0.005)

y_speed = 0
x_speed = 0


# initial position of servo1
def initGrip():
    Board.setPWMServoPulse(1, servo1, 800)
    AK.setPitchRangeMoving((0, 8, 10), -90, 90, 0, 1500)  # camera looking down


# set buzzer
def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)


# turn off the motor
def MotorStop():
    Board.setMotor(1, 0)
    Board.setMotor(2, 0)
    Board.setMotor(3, 0)
    Board.setMotor(4, 0)


# Function to set LED color: based on value in r, g,b- simplified compared to OG code
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


# from colorSorting script
# count = 0
# color_list = []
# get_roi = False
# detect_color = 'None'
# start_pick_up = False
# start_count_t1 = True

_stop = False
__isRunning = False
detect_color = 'None'
start_pick_up = False


# variable reset
def reset():
    global _stop
    global __isRunning
    global detect_color
    global start_pick_up
    global __target_color
    global x_dis, y_dis
    global enableWheel

    x_dis = 1500
    y_dis = 860
    x_pid.clear()
    y_pid.clear()
    go_pid.clear()
    about_pid.clear()
    _stop = False
    enableWheel = False
    __target_color = ()
    detect_color = 'None'
    start_pick_up = False


# initialize program
def init():
    print("ColorTracking Init")
    # print("ColorSorting Init")
    # the light is turned off by default after the ultrasonic wave is turned on
    HWSONAR.setRGBMode(0)
    HWSONAR.setPixelColor(0, Board.PixelColor(0, 0, 0))
    HWSONAR.setPixelColor(1, Board.PixelColor(0, 0, 0))
    HWSONAR.show()
    load_config()
    reset()
    initGrip()


# start program
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorTracking Start")


# stop program
def stop():
    global _stop
    global __isRunning, auto_controller
    _stop = True
    reset()
    initGrip()
    MotorStop()
    # autonomous_controller.stop
    __isRunning = False
    set_rgb('None')
    print("ColorTracking Stop")


# exit program
def exit():
    global _stop
    global __isRunning, auto_controller
    _stop = True
    reset()
    initMove()
    MotorStop()
    __isRunning = False
    set_rgb('None')

    if auto_controller is not None:
        auto_controller.stop()  # Stop ultrasonic navigation

    print("ColorTracking Exit")


# set body to follow tracking()
enableWheel = False  # Default: Wheel tracking is off


def setWheel(Wheel=0, detected_color=None, center_x=None, center_y=None, img_w=None, img_h=None):
    global enableWheel, auto_controller

    # Stop ultrasonic tracking when a valid color is detected
    if auto_controller is not None:
        auto_controller.tracking_mode = True

        # Enable wheel tracking only if a valid color is detected
    if detected_color in __target_color:
        # Only start tracking if the object is within the center threshold
        if abs(center_x - img_w / 2.0) < 20 and abs(center_y - img_h / 2.0) < 20:
            enableWheel = True  # Start tracking
            print(f"Tracking Enabled for {detected_color}")
        else:
            enableWheel = False  # Object not centered yet

    else:  # No valid color detected, stop movement
        enableWheel = False
        print("No color detected. Stopping tracking.")
        MotorStop()

    return (True, ())


# variables for tracking()
rect = None
size = (640, 480)


# Color detection Function run() from ColorTracking source code
def run(img):
    global rect
    global __isRunning
    global enableWheel
    global detect_color
    global start_pick_up
    global img_h, img_w
    global x_dis, y_dis
    global x_speed, y_speed

    global auto_controller  # references AutonomousController

    # copy frame
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    if not __isRunning:  # checks to see if __isRunning is False; stops processing if not running
        return img

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # convert the image to lab space

    Motor_ = True
    area_max = 0
    areaMaxContour = 0

    if not start_pick_up:
        for i in lab_data:
            if i in __target_color:
                detect_color = i
                frame_mask = cv2.inRange(frame_lab,
                                         (lab_data[detect_color]['min'][0],
                                          lab_data[detect_color]['min'][1],
                                          lab_data[detect_color]['min'][2]),
                                         (lab_data[detect_color]['max'][0],
                                          lab_data[detect_color]['max'][1],
                                          lab_data[detect_color]['max'][
                                              2]))  # preform bitwise operation on the og image and mask
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # open operation
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # close operation
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # find the contour
                areaMaxContour, area_max = getAreaMaxContour(contours)  # find the largest contour

        if area_max > 1000:  # the max area is found; valid object is detected
            (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour)  # get the min circumscribed circle
            center_x = int(Misc.map(center_x, 0, size[0], 0, img_w))
            center_y = int(Misc.map(center_y, 0, size[1], 0, img_h))
            radius = int(Misc.map(radius, 0, size[0], 0, img_w))

            if radius > 100:
                return img  # ignore large objects

            # Stop ultrasonic navigation if a color is detected
            if auto_controller is not None:
                auto_controller.tracking_mode = True

            setWheel(Wheel=1, detected_color=detect_color, center_x=center_x, center_y=center_y, img_w=img_w,
                     img_h=img_h)

            rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(rect))
            cv2.circle(img, (int(center_x), int(center_y)), int(radius), range_rgb[detect_color], 2)

            # if __isRunning:   #checks value
            # if enableWheel == True:   #check whether veh tracking fnx is enabled;
            # enableWheel = True,to enable veh tracking
            # Motor_ = True

            if __isRunning and enableWheel:  # Cleaner check; would have to fix indentations if this is applied.
                Motor_ = True  # allow movement

                # PID control for tracking
                if abs(center_x - img_w / 2.0) < 15:  # object is already centered horizontally, so no need to move
                    about_pid.SetPoint = center_x
                else:
                    about_pid.SetPoint = img_w / 2.0  # set point at center

                about_pid.update(center_x)  # update PID for x
                x_speed = max(-100, min(100, -int(about_pid.output)))
                # get constrain PID output value
                # x_speed = -100 if x_speed < -100 else x_speed
                # x_speed = 100 if x_speed > 100 else x_speed

                if abs(center_y - img_h / 2.0) < 10:  # Object is centered vertically, so no need to move
                    go_pid.SetPoint = center_y
                else:
                    go_pid.SetPoint = img_h / 2.0

                go_pid.update(center_y)  # update PID for  y
                y_speed = max(-100, min(100, int(go_pid.output)))
                # get constrain PID output value
                # y_speed = -100 if y_speed < -100 else y_speed
                # y_speed = 100 if y_speed > 100 else y_speed

                # speed synthesis; assign motor speeds
                speed_1 = int(y_speed + x_speed)
                speed_2 = int(y_speed - x_speed)
                speed_3 = int(y_speed - x_speed)
                speed_4 = int(y_speed + x_speed)

                Board.setMotor(1, speed_1)
                Board.setMotor(2, speed_2)
                Board.setMotor(3, speed_3)
                Board.setMotor(4, speed_4)

            else:  # stop movement when tracking is disabled
                if Motor_:
                    MotorStop()
                    Motor_ = False

                # adjust arm positions
                x_pid.SetPoint = img_w / 2.0  # set
                x_pid.update(center_x)  # current
                dx = x_pid.output
                x_dis += int(dx)  # output
                x_dis = 500 if x_dis < 500 else x_dis
                x_dis = 2500 if x_dis > 2500 else x_dis

                y_pid.SetPoint = img_h / 2.0  # set
                y_pid.update(center_y)  # current
                dy = y_pid.output
                y_dis += int(dy)  # output
                y_dis = 500 if y_dis < 500 else y_dis
                y_dis = 2500 if y_dis > 2500 else y_dis

                Board.setPWMServosPulse([20, 2, 3, int(y_dis), 6, int(x_dis)])
        else:
            # stop motors if no object is detected
            if Motor_:
                MotorStop()
                Motor_ = False
    return img


def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--Wheel', type=int, default=0, help='0 or 1')
    opt = parser.parse_args()
    return opt


def Stop(signum, frame):
    global __isRunning

    __isRunning = False
    print('Closing...')
    MotorStop()  # turn off all motors


def pick_up_object():
    global img_h, img_w
    global x_dis, y_dis

    print("Picking up object...")

    # Use PID to move the arm toward detected position
    x_pid.SetPoint = img_w / 2.0  # Center of camera view
    x_pid.update(x_dis)
    x_dis += int(x_pid.output)
    x_dis = max(500, min(2500, x_dis))  # Limit movement range

    y_pid.SetPoint = img_h / 2.0  # Center of camera view
    y_pid.update(y_dis)
    y_dis += int(y_pid.output)
    y_dis = max(500, min(2500, y_dis))  # Limit movement range

    # Move the arm to the detected position
    AK.setPitchRangeMoving((x_dis, y_dis, 10), -90, -90, 0, 1000)
    time.sleep(1.5)

    # Close the gripper
    Board.setPWMServoPulse(1, 1500, 500)
    time.sleep(1.5)

    print("Object picked up!")


# Arm End positions
sorting_positions = {
    'red': (-15, 14, 2),  # Move Left
    'green': (-18, 9, 3),  # Move Center
    'blue': (-18, 0, 2),  # Move Right
    'capture': (0, 16.5, 2)
}


def color_sorting(detected_color):
    if detected_color in sorting_positions:
        target_pos = sorting_positions[detected_color]
        print(f"Sorting {detected_color} to {target_pos}")

        # raise the robotic arm
        # AK.setPitchRangeMoving(target_pos, 0, -90, 90, 1500)
        # time.sleep(1.5)

        # Move arm to sorting location
        AK.setPitchRangeMoving(target_pos, -90, -90, 0, 800)
        time.sleep(1)

        # Release object
        Board.setPWMServoPulse(1, 1800, 500)
        time.sleep(1)

        # Return arm to starting position
        AK.setPitchRangeMoving((0, 8, 10), -90, -90, 0, 1000)
        time.sleep(1)
    print("Sorting complete!")


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
        self.tracking_mode = False  # switch to color tracking

    def object_detected(self):
        dist = self.sensor_mod.getDistance()
        if not self.tracking_mode:  # only use sonar if no color is detected

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
                # self.motion_controller.stop()
            else:
                print("No obstacle nearby, moving forward")
                self.motion_controller.move(40, 90)

    def Stop(self):
        self.running = False
        self.motion_controller.stop()
        print("Autonomous movement stopped")

    # signal.signal(signal.SIGINT, Stop)

    def start(self):
        try:
            while self.running:
                if not self.tracking_mode:  # Use sonar guidance if no color detected
                    self.object_detected()
                else:
                    # Process frames using detect_color()
                    ret, img = cap.read()
                    if ret:
                        processed_frame, detected_color = detect_color(img)
                    setWheel(Wheel=1, detected_color=detected_color)
                    frame_resize = cv2.resize(processed_frame, (320, 240))
                    cv2.imshow('frame', frame_resize)
                    if detected_color:
                        sorting_thread = threading.Thread(target=color_sorting, args=(detected_color,))
                        sorting_thread.start()

                time.sleep(0.5)
        except KeyboardInterrupt:
            self.stop()


# Main Execution (Multithreading)
if __name__ == '__main__':
    load_config()

    # Create instances
    auto_controller = AutonomousController()

    # Start obstacle avoidance in a separate thread
    obstacle_thread = threading.Thread(target=auto_controller.start)
    obstacle_thread.daemon = True
    obstacle_thread.start()

    # start video capture for color detection
    cap = cv2.VideoCapture('http://127.0.0.1:8080?action=stream')

    # process frames for color detection
    while True:
        ret, img = cap.read()
        if ret:
            frame = img.copy()

            # Process frame for color detection
            processed_frame, detected_color = detect_color(frame)

            # Start wheel tracking if a valid color is detected
            setWheel(Wheel=1, detected_color=detected_color)

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
