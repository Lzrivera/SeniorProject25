import time
from threading import Thread
from HiwonderSDK.mecanum import MecanumChassis
from Sonar import Sonar
from ArmIK import ArmIK
from HiwonderSDK.Board import setMotor, setPWMServoPulse
from PID import PID

class SensorModule:
    def __init__(self):
        self.sonar = Sonar()

    def get_distance(self):
        return self.sonar.getDistance()

class MotionController:
    def __init__(self):
        self.chassis = MecanumChassis()

    def move_forward(self, velocity=50):
        self.chassis.set_velocity(velocity, 90, 0)

    def turn(self, angle=90, velocity=50):
        if angle > 0:
            self.chassis.set_velocity(0, 90, 0.3)  # Turn clockwise
        else:
            self.chassis.set_velocity(0, 90, -0.3)  # Turn counterclockwise
        time.sleep(3)
        self.stop()

    def stop(self):
        self.chassis.set_velocity(0, 0, 0)

    def slant(self, velocity=50, direction=45):
        self.chassis.set_velocity(velocity, direction, 0)

class ArmController:
    def __init__(self):
        self.arm = ArmIK()

    def pick_up_object(self):
        print("Picking up object...")
        self.arm.setPitchRangeMoving((10, 10, 10), -90, -90, 90, 1500)  # Example position
        setPWMServoPulse(1, 1500, 800)  # Close gripper
        time.sleep(1)

    def move_object_out_of_way(self):
        print("Moving object...")
        self.arm.setPitchRangeMoving((10, -10, 10), -90, -90, 90, 1500)  # Example move
        time.sleep(1)

    def reset_arm(self):
        self.arm.setPitchRangeMoving((0, 0, 10), 0, -90, 90, 1500)  # Reset arm
        time.sleep(1)

class MainAutonomousController:
    def __init__(self):
        self.sensor_module = SensorModule()
        self.motion_controller = MotionController()
        self.arm_controller = ArmController()
        self.running = True

    def avoid_obstacle(self):
        distance = self.sensor_module.get_distance()
        if distance < 30:  # Threshold for obstacle
            print("Obstacle detected! Stopping.")
            self.motion_controller.stop()
            self.arm_controller.pick_up_object()
            self.arm_controller.move_object_out_of_way()
            self.arm_controller.reset_arm()

    def perform_dynamic_movements(self):
        print("Performing dynamic movements...")
        self.motion_controller.slant(velocity=50, direction=45)  # Example slant movement
        time.sleep(2)
        self.motion_controller.turn(angle=90)  # Example turn movement
        time.sleep(2)
        self.motion_controller.move_forward()  # Resume forward motion

    def start(self):
        try:
            while self.running:
                self.avoid_obstacle()
                self.perform_dynamic_movements()
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        self.running = False
        self.motion_controller.stop()
        print("Autonomous movement stopped.")

if __name__ == "__main__":
    controller = MainAutonomousController()
    controller.start()
