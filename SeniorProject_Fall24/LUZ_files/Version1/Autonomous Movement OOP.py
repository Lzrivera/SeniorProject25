Python 3.9.5 (tags/v3.9.5:0a7dcbd, May  3 2021, 17:27:52) [MSC v.1928 64 bit (AMD64)] on win32
Type "help", "copyright", "credits" or "license()" for more information.
>>> import time
from threading import Thread
from HiwonderSDK.Board import setMotor, getBattery, setBuzzer
from HiwonderSDK.mecanum import MecanumChassis
from Camera import Camera
from PID import PID
from Sonar import Sonar
from yaml_handle import get_yaml_data

class SensorModule:
    def __init__(self):
        self.sonar = Sonar()
        self.camera = Camera()
        self.camera.camera_open()

    def get_distance(self):
        return self.sonar.getDistance()

    def get_camera_frame(self):
        return self.camera.frame

    def close(self):
        self.camera.camera_close()

class MotionController:
    def __init__(self):
        self.chassis = MecanumChassis()

    def move(self, velocity, direction, angular_rate=0):
        self.chassis.set_velocity(velocity, direction, angular_rate)

    def stop(self):
        self.chassis.set_velocity(0, 0, 0)

class VisionProcessor:
    def __init__(self, target_color='red'):
        self.target_color = target_color
        self.pid = PID(P=0.5, I=0.1, D=0.05)

    def process_frame(self, frame):
        # Example of applying a PID-controlled adjustment
        # In reality, this would include actual image processing
        if frame is not None:
            # Simulate tracking logic
            print(f"Processing frame for {self.target_color}")
        return frame

class MainAutonomousController:
    def __init__(self):
        self.sensor_module = SensorModule()
        self.motion_controller = MotionController()
        self.vision_processor = VisionProcessor()
        self.running = True

    def avoid_obstacles(self):
        distance = self.sensor_module.get_distance()
        if distance < 30:
            print("Obstacle detected! Stopping.")
            self.motion_controller.stop()

    def track_color(self):
        frame = self.sensor_module.get_camera_frame()
        if frame is not None:
            processed_frame = self.vision_processor.process_frame(frame)
            print("Tracking color...")

    def start(self):
        try:
            while self.running:
                self.avoid_obstacles()
                self.track_color()
                self.motion_controller.move(velocity=50, direction=90)
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        self.running = False
        self.motion_controller.stop()
        self.sensor_module.close()
        print("Autonomous movement stopped.")

if __name__ == "__main__":
    controller = MainAutonomousController()
    controller.start()
