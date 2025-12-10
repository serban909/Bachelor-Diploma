from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

class HardwareAdapter:
    def __init__(self, left_motor_port=Port.A, right_motor_port=Port.C, ultrasonic_sensor_port=Port.S4, left_ultrasonic_port=Port.S1, left_color_port=Port.S2, right_color_port=Port.S3, wheel_diameter=55.5, axle_track=104):
        self.ev3 = EV3Brick()
        self.left_motor = Motor(left_motor_port)
        self.right_motor = Motor(right_motor_port)
        self.ultrasonic_sensor = UltrasonicSensor(ultrasonic_sensor_port)
        self.left_ultrasonic_sensor = UltrasonicSensor(left_ultrasonic_port)
        self.left_color_sensor = ColorSensor(left_color_port)
        self.right_color_sensor = ColorSensor(right_color_port)
        self.robot = DriveBase(self.left_motor, self.right_motor, wheel_diameter=wheel_diameter, axle_track=axle_track)

    def drive_forward(self, speed_mm_s):
        self.robot.drive(speed_mm_s, 0)

    def drive_turn_rate(self, speed_mm_s, turn_rate):
        self.robot.drive(speed_mm_s, turn_rate)

    def stop(self, brake=True):
        if brake:
            self.robot.stop()
        else:
            self.robot.stop()
            
    def straight(self, distance_mm):
        self.robot.straight(distance_mm)
        
    def turn_angle(self, angle_deg):
        self.robot.turn(angle_deg)
        
    def left_reflection(self):
        if self.left_color_sensor is None:
            return 50
        return self.left_color_sensor.reflection()
    
    def right_reflection(self):
        if self.right_color_sensor is None:
            return 50
        return self.right_color_sensor.reflection()
    
    def distance_mm(self):
        return self.ultrasonic_sensor.distance()
    
    def left_wall_distance_mm(self):
        """Get distance to left wall from side-facing sensor"""
        return self.left_ultrasonic_sensor.distance()