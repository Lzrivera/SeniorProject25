import os
import sys

sys.path.append('/home/pi/MasterPi/')
#sys.path.append('/home/pi/MasterPi/ArmIK')

import Camera
import argparse

import cv2
import time
import threading
import yaml_handle
import signal
import math
import numpy as np
import HiwonderSDK.PID as PID
import HiwonderSDK.Sonar as Sonar
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *
#sys.path.append('/home/pi/MasterPi/')
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
    global lab_data, servo_data
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

#closing angle of the gripper when gripping
servo1 = 1500

#set distance
x_dis = 1500
y_dis = 860

 # pid initalizing
x_pid = PID.PID(P=0.28, I=0.03, D=0.03) 
y_pid = PID.PID(P=0.28, I=0.03, D=0.03)

go_pid = PID.PID(P=0.28, I=0.1, D=0.05)
about_pid = PID.PID(P=0.35, I=0.08, D=0.005)

y_speed = 0
x_speed = 0

# inital postion of servo1
def initGrip():
    Board.setPWMServoPulse(1, servo1, 800)
    AK.setPitchRangeMoving((0, 8, 10), -90, 90, 0, 1500)


 
 #turn off the motor
def MotorStop():
    Board.setMotor(1, 0) 
    Board.setMotor(2, 0)
    Board.setMotor(3, 0)
    Board.setMotor(4, 0)

# Function to set LED color: based on value in r, g,b 
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

detect_color = 'None'
start_pick_up = False
start_count_t1 = True

#variables for tracking()
rect = None
size = (640, 480)

#variables for move()
rotation_angle = 0
unreachable = False
world_X, world_Y = 0, 0

#variables for run()
t1 = 0
roi = ()
center_list = []
last_x, last_y = 0, 0
draw_color = range_rgb["black"]

"""#variable reset
def reset(): 
    global _stop
    global count
    global get_roi
    global color_list
    global detect_color
    global start_pick_up
    global start_count_t1
    
    #from tracking
    global _isRunning
    global _target_color
    global x_dis,y_dis
    global enableWheel
    x_dis = 1500
    y_dis = 860
    x_pid.clear()
    y_pid.clear()
    go_pid.clear()
    about_pid.clear()
    enableWheel = False
    __target_color = ()
    
    
    #from run()
    count = 0
    _stop = False
    color_list = []
    get_roi = False
    detect_color = 'None'
    start_pick_up = False
    start_count_t1 = True"""

#set body to follow tracking()
enableWheel = True
def setWheel(Wheel = 1,):
    global enableWheel
    if Wheel :
        enableWheel = False
        MotorStop()
        initGrip()
    else:
        enableWheel = True
        Board.setPWMServoPulse(1, 1500, 800)
        AK.setPitchRangeMoving((0, 7, 12), -50, -90, 0, 1500)

    return (True, ())


# Color Detection Function run() and move() together from source code
def detect_color(img):
    global roi
    global rect
    global count
    global _stop
    global get_roi
    global center_list

    global unreachable
    global start_count_t1, t1
    global detect_color, draw_color, color_list
    global start_pick_up
    global rotation_angle
    global last_x, last_y
    global world_X, world_Y
    global img_h, img_w
    global x_dis, y_dis
    global x_speed,y_speed

#run() code
    global __target_color
    img_copy = img.copy() 
    img_h, img_w = img.shape[:2]
    
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB) #convert the image to lab space


    max_area = 0
    area_max_contour = 0 
    detected_color = None
    Motor_ = True
    #if not start_pick_up:
    for color in __target_color:
        mask = cv2.inRange(frame_lab, 
                           (lab_data[color]['min'][0], lab_data[color]['min'][1],
                            lab_data[color]['min'][2]),
                           (lab_data[color]['max'][0], lab_data[color]['max'][1],
                            lab_data[color]['max'][2]))
        opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  #open operation
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  #close operation
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        area_max_contour, area_max = getAreaMaxContour(contours) #find the maximum contour

        if area_max_contour is not None and area_max > max_area:
            max_area = area_max
            detected_color = color
            
    if max_area > 1000:  #the maximum area has been found
        (center_x, center_y), radius = cv2.minEnclosingCircle(area_max_contour)  # center_x = int(Misc.map(center_x, 0, size[0], 0, img_w))
        center_y = int(Misc.map(center_y, 0, size[1], 0, img_h))
        radius = int(Misc.map(radius, 0, size[0], 0, img_w))
        if radius > 100:
            return img
        
        rect = cv2.minAreaRect(area_max_contour)
        box = np.int0(cv2.boxPoints(rect))
                
        cv2.circle(img, (int(center_x), int(center_y)), int(radius), range_rgb[detect_color], 2)
        
        if enableWheel == True:
            Motor_ = True
            if abs(center_x - img_w/2.0) < 15: #the movement is small, so no need to move
                about_pid.SetPoint = center_x
            else:
                about_pid.SetPoint = img_w/2.0 # set
            about_pid.update(center_x) #current 
            x_speed = -int(about_pid.output)  # get PID output value
            x_speed = -100 if x_speed < -100 else x_speed
            x_speed = 100 if x_speed > 100 else x_speed
            
            if abs(center_y - img_h/2.0) < 10: #the movement is small, so no need to move
                go_pid.SetPoint = center_y
            else:
                go_pid.SetPoint = img_h/2.0  
            go_pid.update(center_y)
            y_speed = int(go_pid.output)#get PID output value
            y_speed = -100 if y_speed < -100 else y_speed
            y_speed = 100 if y_speed > 100 else y_speed
            
            speed_1 = int(y_speed + x_speed) # speed synthesis
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
            x_pid.SetPoint = img_w / 2.0  #set 
            x_pid.update(center_x)  #current
            dx = x_pid.output
            x_dis += int(dx)  #output
            x_dis = 500 if x_dis < 500 else x_dis
            x_dis = 2500 if x_dis > 2500 else x_dis
            
            y_pid.SetPoint = img_h / 2.0  #set
            y_pid.update(center_y)  #current 
            dy = y_pid.output
            y_dis += int(dy)  #output 
            y_dis = 500 if y_dis < 500 else y_dis
            y_dis = 2500 if y_dis > 2500 else y_dis
                             
            Board.setPWMServosPulse([20, 2, 3,int(y_dis), 6,int(x_dis)])

        
        if not start_pick_up:
            if color_area_max == 'red':  # red is the largest
                color = 1
            elif color_area_max == 'green':  # green max
                color = 2
            elif color_area_max == 'blue':  #blue max
                color = 3
            else:
                color = 0
            color_list.append(color)
            if len(color_list) == 3:  # multiple judgements
                #get the average
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
            set_rgb('None')
            draw_color =(0, 0, 0)
    if detected_color:
        cv2.putText(img, f"Color: {detected_color}", (10, img.shape[0] - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, draw_color, 2) # print the detected color on
    #the screen. other code possbility range_rgb[detected_color], 2)
        set_rgb(detected_color) #set the color of the light on the expansion board to the same as the dectected color
        #setBuzzer(0.1)     # set buzzer to sound for 0.1 seconds


    return img, detected_color

def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--Wheel', type=int, default=0, help='0 or 1')
    opt = parser.parse_args()
    return opt


# Sorting Function/tracking function
"""def color_sorting(detected_color):
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
        time.sleep(1)"""

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
        
    def Stop(self):
        self.running = False
        self.motion_controller.stop()
        cv2.destroyAllWindows()
        MotorStop()
        initGrip()
        print("Autonomous movement stopped")
    
    signal.signal(signal.SIGINT, Stop)

    def start(self):
        try:
            while self.running:
                self.object_detected() and self.detect_color(img)
                time.sleep(0.5)
                opt = parse_opt()
                setWheel(**vars(opt))
                __target_color =("red")
                signal.signal(signal.SIGINT, Stop)
                
                
                        
        except KeyboardInterrupt:
            self.Stop()


# Main Execution (Multithreading)
if __name__ == '__main__':
    test = AutonomousController()
    test.start()
    cap = cv2.VideoCapture('http://127.0.0.1:8080?action=stream')
    ret,img = cap.read()
    while enableWheel:
        if ret:
            frame = img.copy()
            Frame = run(frame)  
            frame_resize = cv2.resize(Frame, (320, 240))
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                break
    else:
        time.sleep(0.01)
    cv2.destroyAllWindows()

    

#     # Create instances
#     auto_controller = AutonomousController()
# 
#     # Start obstacle avoidance in a separate thread
#     obstacle_thread = threading.Thread(target=auto_controller.start)
#     obstacle_thread.daemon = True
#     obstacle_thread.start()
# 
#     cap = cv2.VideoCapture('http://127.0.0.1:8080?action=stream')
# 
#     while True:
#         ret, img = cap.read()
#         if ret:
#             frame = img.copy()
#             processed_frame, detected_color = detect_color(frame)
# 
#             frame_resize = cv2.resize(processed_frame, (320, 240))
#             cv2.imshow('frame', frame_resize)
# 
#             if detected_color:
#                 sorting_thread = threading.Thread(target=color_sorting, args=(detected_color,))
#                 sorting_thread.start()
# 
#             key = cv2.waitKey(1)
#             if key == 27:  # Press 'ESC' to exit
#                 break
#         else:
#             time.sleep(0.01)

    
