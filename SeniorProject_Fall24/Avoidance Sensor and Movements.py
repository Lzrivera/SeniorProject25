import sys
sys.path.append('/home/pi/MasterPi/')
import time
import signal
import HiwonderSDK.mecanum as mecanum
import cv2
import Camera
import numpy as np
import pandas as pd
import HiwonderSDK.Sonar as Sonar
import HiwonderSDK.Board as Board
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
from ArmIK.InverseKinematics import *


AK = ArmIK()
chassis = mecanum.MecanumChassis()

start = True

def Stop(signum, frame):
    global start
    start = False
    chassis.set_velocity(0,0,0)  
    
signal.signal(signal.SIGINT, Stop)

HWSONAR = None
Threshold = 30.0
TextColor = (0, 255, 255)
TextSize = 12

speed = 40
__isRunning = False
__until = 0


# to close the gripper
servo1 = 1500

# Inital position 
def initMove():
    chassis.set_velocity(0,0,0)
    Board.setPWMServoPulse(1, servo1, 300)
    AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90, 1500)

# variable reset
def reset():
    global __isRunning
    global Threshold
    global speed
    global stopMotor
    global turn
    global forward
    global old_speed
    
    speed = 40
    old_speed = 0
    Threshold = 30.0
    turn = True
    forward = True
    stopMotor = True
    __isRunning = False
    
# app initialization call
def init():
    print("Avoidance Init")
    initMove()
    reset()
    
__isRunning = False
# app starts gameplay call
def start():
    global __isRunning
    global stopMotor
    global forward
    global turn
    
    turn = True
    forward = True
    stopMotor = True
    __isRunning = True
    print("Avoidance Start")

# app stops gameplay call
def stop():
    global __isRunning
    __isRunning = False
    chassis.set_velocity(0,0,0)
    print("Avoidance Stop")

# app exit gameplay call
def exit():
    global __isRunning
    __isRunning = False
    chassis.set_velocity(0,0,0)
    HWSONAR.setPixelColor(0, Board.PixelColor(0, 0, 0))
    HWSONAR.setPixelColor(1, Board.PixelColor(0, 0, 0))
    print("Avoidance Exit")

# set obstacle avoidance speed
def setSpeed(args):
    global speed
    speed = int(args[0])
    return (True, ())
 
# set obstacle avoidance threshold     
def setThreshold(args):
    global Threshold
    Threshold = args[0]
    return (True, (Threshold,))

# get the current obstacle avoidance threshold    
def getThreshold(args):
    global Threshold
    return (True, (Threshold,))


turn = True
forward = True
old_speed = 0
stopMotor = True
distance_data = []

def run(img):
    global turn
    global speed
    global __until
    global __isRunning
    global HWSONAR
    global Threshold
    global distance_data
    global stopMotor
    global forward
    global old_speed
    
    dist = HWSONAR.getDistance() / 10.0

    distance_data.append(dist)
    data = pd.DataFrame(distance_data)
    data_ = data.copy()
    u = data_.mean()  # calculate the mean
    std = data_.std()  # calculate the standard deviation

    data_c = data[np.abs(data - u) <= std]
    distance = data_c.mean()[0]

    if len(distance_data) == 5:
        distance_data.remove(distance_data[0])

    if __isRunning:   
        if speed != old_speed:   # set the same speed value 
            old_speed = speed
            chassis.set_velocity(speed,90,0)
            
        if distance <= Threshold:   # check weather the distance threshold is reached 
            if turn:
                turn = False
                forward = True
                stopMotor = True
                chassis.set_velocity(0,90,-0.5)
                time.sleep(0.5)
            
        else:
            if forward:
                turn = True
                forward = False
                stopMotor = True
                chassis.set_velocity(speed,90,0)
    else:
        if stopMotor:
            stopMotor = False
            chassis.set_velocity(0,0,0)  
        turn = True
        forward = True
        time.sleep(0.03)

    return cv2.putText(img, "Dist:%.1fcm"%distance, (30, 480-30), cv2.FONT_HERSHEY_SIMPLEX, 1.2, TextColor, 2)  # 把超声波测距值打印在画面上


#关闭前处理
def Stop(signum, frame):
    global __isRunning
    
    __isRunning = False
   # print('关闭中...')
    chassis.set_velocity(0,0,0)  # 关闭所有电机

if __name__ == '__main__':
    init()
    start()
    HWSONAR = Sonar.Sonar()
    signal.signal(signal.SIGINT, Stop)
    cap = cv2.VideoCapture('http://127.0.0.1:8080?action=stream')
    while __isRunning:
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
    cv2.destroyAllWindows()
start = False