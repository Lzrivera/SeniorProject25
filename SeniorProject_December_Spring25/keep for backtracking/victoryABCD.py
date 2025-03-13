import os 
import sys
sys.path.append('/home/pi/MasterPi/')
import cv2
import time
import signal
import Camera
import numpy as np
import pandas as pd
import HiwonderSDK.Sonar as Sonar
import threading
import yaml_handle
sys.path.append('/home/pi/MasterPi/ArmIK')
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
import HiwonderSDK.mecanum as mecanum

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()
HWSONAR = Sonar.Sonar()

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
    'orange': (255,165,0),
    'purple': (160,32,240),
    'yellow': (225,255,0) ,
}

lab_data = None #data from camera
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

__target_color = ('red', 'green', 'blue') #, 'yellow','purple', 'orange' ) additional colors for later
def setTargetColor(target_color):
    global __target_color

    __target_color = target_color
    return (True, ())

# find the wheel with the largest areaparameter table
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # traverse all controus
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积 calculate the contour area
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:  # the contour with the largest area is valid only when the area
                #is greater than 300 to filter out interference
                area_max_contour = c

    return area_max_contour, contour_area_max  # return the largest contour

# closing agle of the gripper when gripping
servo1 = 1500

# initial position
def initMove():
    Board.setPWMServoPulse(1, servo1, 800)
    AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90, 1500)


# set buzzer
def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)
   

# set the rgb light color of the expansion board to match the color to be tracked
def set_rgb(color):
    if color == "red":
        Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
        Board.RGB.show()
    elif color == "green":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
        Board.RGB.show()
    elif color == "blue":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
        Board.RGB.show()
    else:
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
        Board.RGB.show()
    """elif color == "orange":
        Board.RGB.setPixelColor(0, Board.PixelColor(255, 165, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(255, 165, 0))
        Board.RGB.show()
    elif color == "purple":
        Board.RGB.setPixelColor(0, Board.PixelColor(160, 32, 240))
        Board.RGB.setPixelColor(1, Board.PixelColor(160, 32, 240))
        Board.RGB.show()
    elif color == "yellow":
        Board.RGB.setPixelColor(0, Board.PixelColor(255, 255, 255))
        Board.RGB.setPixelColor(1, Board.PixelColor(255, 255, 255))
        Board.RGB.show()"""


count = 0
_stop = False
color_list = []
get_roi = False
__isRunning = False
detect_color = 'None'
start_pick_up = False
start_count_t1 = True

#variable reset
def reset(): 
    global _stop
    global count
    global get_roi
    global color_list
    global detect_color
    global start_pick_up
    global __target_color
    global start_count_t1
    
    count = 0
    _stop = False
    color_list = []
    get_roi = False
    detect_color = 'None'
    start_pick_up = False
    start_count_t1 = True



rect = None
size = (640, 480)
rotation_angle = 0
unreachable = False
world_X, world_Y = 0, 0
def Armmove():
    global rect
    global _stop
    global get_roi
    global __isRunning
    global unreachable
    global detect_color
    global start_pick_up
    global rotation_angle
    global world_X, world_Y
    
    # place coordinates
    coordinate = {
        'red':   (-15, 14, 2),
        'green': (-18, 9,  3),
        'blue':  (-18, 0, 2),
        'capture': (0, 16.5, 2)
    }

    while True:
        if __isRunning:
            if detect_color != 'None' and start_pick_up:  #detect color blocks
                
                set_rgb(detect_color) # set the color of the light on the expansion board to
                #the same as the dectected color
                setBuzzer(0.1)     #set buzzer to sound for 0.1 seconds
                
                AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90, 1500) # 机械臂抬起来 raise the robot arm
                time.sleep(1.5)
                if not __isRunning:  # check whether to stop playing
                    continue
                
                Board.setPWMServoPulse(1, 2000, 500) #open the claws
                time.sleep(1.5)
                if not __isRunning:
                    continue
                
                Board.setPWMServoPulse(1, 1500, 500) # close the claws
                time.sleep(1.5)
                if not __isRunning:
                    continue
                
                if detect_color == 'red':       #accorind to the deteced color the robot arm rotates to the 
                    #corresponding angle
                    #add colordetect head nodding if we are able to get the arm to pick block functioning. for a later date
                    Board.setPWMServoPulse(6, 1900, 500)
                    time.sleep(0.5)
                elif detect_color == 'green':
                    Board.setPWMServoPulse(6, 2100, 800)
                    time.sleep(0.8)
                elif detect_color == 'blue':
                    Board.setPWMServoPulse(6, 2500, 1500)
                    time.sleep(1.5)
                    
                if not __isRunning:
                    continue
                result = AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 8), -90, -90, 0) #  run to the coordinates of the responding color
                if result == False:
                    unreachable = True
                else:
                    unreachable = False
                    time.sleep(result[2]/1000) # if the specified location can be reached, the running time is obtained
                if not __isRunning:
                    continue
                AK.setPitchRangeMoving((coordinate[detect_color]), -90, -90, 0, 500)  #  place it at the coordinates corresponding to the detect color
                time.sleep(0.5)
                if not __isRunning:
                    continue
                Board.setPWMServoPulse(1, 1800, 500) # open claw
                time.sleep(0.5)
                if not __isRunning:
                    continue
                AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 8), -90, -90, 0, 800) # run to the coordinates of the corresponding color
                time.sleep(0.8)
                if not __isRunning:
                    continue
                Board.setPWMServosPulse([1200, 4, 1,1500, 3,515, 4,2170, 5,945]) # the robot arm is reset
                time.sleep(1.2)
                if detect_color == 'red':
                    Board.setPWMServoPulse(6, 1500, 500)
                    time.sleep(0.5)
                elif detect_color == 'green':
                    Board.setPWMServoPulse(6, 1500, 800)
                    time.sleep(0.8)
                elif detect_color == 'blue':
                    Board.setPWMServoPulse(6, 1500, 1500)
                    time.sleep(1.5)
                """elif detect_color == 'yellow':
                    Board.setPWMServoPulse(6, 1500, 1500)
                    time.sleep(1.5)
                elif detect_color == 'purple':
                    Board.setPWMServoPulse(6, 1500, 1500)
                    time.sleep(1.5)
                elif detect_color == 'orange':
                    Board.setPWMServoPulse(6, 1500, 1500)
                    time.sleep(1.5)"""
                    
                AK.setPitchRangeMoving((0, 8, 10), -90, -90, 0, 1000)
                time.sleep(1)
                
                detect_color = 'None'
                get_roi = False
                start_pick_up = False
                set_rgb(detect_color)
            else:
                time.sleep(0.01)                
        else:
            if _stop:
                _stop = False
                initMove()
            time.sleep(0.01)
               
                
# run child thread
th = threading.Thread(target=Armmove)
th.setDaemon(True)
th.start()

t1 = 0
roi = ()
center_list = []
last_x, last_y = 0, 0
draw_color = range_rgb["black"]
length = 50
w_start = 200
h_start = 200

def Camerarun(img):
    global roi
    global rect
    global count
    global get_roi
    global center_list
    global unreachable
    global __isRunning
    global start_pick_up
    global last_x, last_y
    global rotation_angle
    global world_X, world_Y
    global start_count_t1, t1
    global detect_color, draw_color, color_list
    
#     if not __isRunning:  # 检测是否开启玩法，没有开启则返回原图像 check whther the gameplay is turned on. if not, return the orginal image
#         return img
# do not need because we removed app gameplay
#    else:
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # convert the image to LAB Space

    color_area_max = None
    max_area = 0
    areaMaxContour_max = 0
    if not start_pick_up:
        for i in lab_data:
            if i in __target_color:
                frame_mask = cv2.inRange(frame_lab,
                                             (lab_data[i]['min'][0],
                                              lab_data[i]['min'][1],
                                              lab_data[i]['min'][2]),
                                             (lab_data[i]['max'][0],
                                              lab_data[i]['max'][1],
                                              lab_data[i]['max'][2]))  # preform bitwise operations on the oringal image and mask
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # open operation
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # close operation
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #find the contour
                areaMaxContour, area_max = getAreaMaxContour(contours)  # find the maximum contour
                if areaMaxContour is not None:
                    if area_max > max_area:  # find the maximum area
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour
        if max_area > 2500:  # the maximum area has been found
            rect = cv2.minAreaRect(areaMaxContour_max)
            box = np.int0(cv2.boxPoints(rect))
            
            cv2.drawContours(img, [box], -1, range_rgb[color_area_max], 2)
            if not start_pick_up:
                if color_area_max == 'red':  # red max
                    color = 1
                elif color_area_max == 'green':  # green max
                    color = 2
                elif color_area_max == 'blue':  # blue max
                    color = 3
                else:
                    color = 0
                color_list.append(color)
                if len(color_list) == 3:  #multiple judgements
                    # get the average
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
                    """elif color == 4: #incorporate at a later time. only focus on rgb for now
                        detect_color = 'orange'
                        draw_color = range_rgb["orange"]
                    elif color == 5:
                        detect_color = 'purple'
                        draw_color = range_rgb["purple"]
                    elif color == 6:
                        detect_color = 'yellow'
                        draw_color = range_rgb["yellow"]"""

        else:
            if not start_pick_up:
                draw_color = (0, 0, 0)
                detect_color = "None"   
    
    cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2) # print the detected color on the screen
    
    return img

# run child thread
cam = threading.Thread(target=Camerarun, daemon=True)
cam.start()

    
Threshold = 80.0

class PixelStrip(object): #needed for turing
    def begin(self):
        print("PixelStrip Intitialized")
        '''self.begin = begin()'''

class SensorMod:
    #Handles distance measurement using Sonar sensor
    def __init__(self):
        self.sonar = Sonar.Sonar()
    
    def getDistance(self):
        #get distance from sonar sensor and print it
        #return self.sonar.getDistance()
        dist =  self.sonar.getDistance() 
        print (dist)
        return dist    

class MotionController:
    #moves chassis forward and stops
    def __init__(self):
        self.chassis = mecanum.MecanumChassis()

    def move(self, velocity, direction, angular_rate=0):
        self.chassis.set_velocity(velocity, direction, angular_rate)
        #move forward in positive velocity, move backward in negative velocity
        #Turn in any direction with 0-360, 

    def stop(self):
        self.chassis.set_velocity(0, 0, 0)
        #stop all motion


class AutonomousController:
    #main control loop for robot
    def __init__(self):
        print("Autonomous Controller Initialized")
        self.sensor_mod = SensorMod()
        self.motion_controller = MotionController()
        self.running = True
    
    def object_detected(self):
        dist = self.sensor_mod.getDistance()
        #better to use if and else statements and get return value for this
        if self.running:
            if dist <= Threshold:
                print("Object detected! Moving backward")
                #moving backward for detected object
                self.motion_controller.move(40,270)
                time.sleep(2)
                
    
                dist = self.sensor_mod.getDistance()       #re-evaulating the distance value after moving backwards
                
                
                if dist >= Threshold:
                    print("Turning")
                    self.motion_controller.move(40,0,.3)
                    time.sleep(1)
                else:
                    print("Obstacle still detected. Moving backwardds again")
                    self.motion_controller.move(40,270)
                    time.sleep(1)
                
         
                print("Moving forward after avoiding obstacle")
                self.motion_controller.move(40,90)
            
                self.motion_controller.stop() #stops moving forward
        
            else:
                print ("obstacle far away, moving closer")
                self.motion_controller.move(40, 90)
        return dist
        
            
    def start(self):
        try:
            while self.running:
                self.motion_controller.move(velocity=80, direction=90)
                #moving forward
                self.object_detected()
                time.sleep(.5)
        except KeyboardInterrupt:
            self.stop()
            
    def stop(self):
        self.running = False
        self.motion_controller.stop()
        #self.sensor_mod.close()
        print("autonomous movement stopped")

if __name__ == '__main__':
    init_thread = threading.Thread(target=initMove) #initial position of arm
    test = AutonomousController()
    test_thread = threading.Thread(target=test.start)
    init_thread.start()
    test_thread.start()
    cap = cv2.VideoCapture('http://127.0.0.1:8080?action=stream')
    while True:
        ret,img = cap.read()
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

    test.running = False
    test_thread.join()
    cv2.destroyAllWindows()
    
    
