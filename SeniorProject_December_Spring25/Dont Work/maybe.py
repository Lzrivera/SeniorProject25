import os

#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import time
import threading
sys.path.append('/home/pi/MasterPi/')
import yaml_handle
import numpy as np
import HiwonderSDK.Sonar as Sonar
import HiwonderSDK.Board as Board
sys.path.append('/home/pi/MasterPi/ArmIK')
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.mecanum as mecanum

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()
Threshold = 80.0

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

lab_data = None
def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

__target_color = ('red', 'green', 'blue')
def setTargetColor(target_color):
    global __target_color
    __target_color = target_color
    return (True, ())

class SensorMod:
    def __init__(self):
        self.sonar = Sonar.Sonar()
    
    def getDistance(self):
        dist = self.sonar.getDistance()
        print(f"Distance: {dist} cm")
        return dist

class MotionController:
    def __init__(self):
        self.chassis = mecanum.MecanumChassis()

    def move(self, velocity, direction, angular_rate=0):
        self.chassis.set_velocity(velocity, direction, angular_rate)

    def stop(self):
        self.chassis.set_velocity(0, 0, 0)

class AutonomousController:
    def __init__(self):
        print("Autonomous Controller Initialized")
        self.sensor_mod = SensorMod()
        self.motion_controller = MotionController()
        self.running = True
        setTargetColor(('red', 'green', 'blue'))
        self.cap = cv2.VideoCapture(0)  # Change based on camera source
    
    def get_detected_color(self, frame):
        detected_color = run(frame)
        return detected_color if isinstance(detected_color, str) else "No Color Detected"
    
    def object_detected(self):
        dist = self.sensor_mod.getDistance()
        if self.running:
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
                print("Obstacle far away, moving closer")
                self.motion_controller.move(40, 90)
        return dist
    
    def process_frame(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.resize(frame, (320, 240))
                detected_color = self.get_detected_color(frame)
                distance = self.sensor_mod.getDistance()
                
                cv2.putText(frame, f"Color: {detected_color}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f"Distance: {distance:.1f} cm", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                cv2.imshow('Processed Frame', frame)
                
                if cv2.waitKey(1) & 0xFF == 27:
                    break
            time.sleep(0.01)
        cv2.destroyAllWindows()
    
    def start(self):
        try:
            threading.Thread(target=self.process_frame, daemon=True).start()
            while self.running:
                self.object_detected()
                time.sleep(0.5)
        except KeyboardInterrupt:
            self.stop()
        finally:
            self.cap.release()
            cv2.destroyAllWindows()
    
    def stop(self):
        self.running = False
        self.motion_controller.stop()
        print("Autonomous movement stopped")

if __name__ == '__main__':
    test = AutonomousController()
    test.start()
