from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from Behavior import Behavior
from PathLogger import PathLogger
from DataExporter import DataExporter
from PIDRegulator import PIDRegulator
from PIFuzzyRegulator import PIFuzzyRegulator

class LaneKeeping(Behavior):
    def __init__(self, target_reflect=40, base_speed=150, kp=1.0, threshold=25, 
                 regulator_type="PID", regulator_kp=1.2, regulator_ki=0.3, regulator_kd=0.05):
        self.target = target_reflect
        self.base_speed = base_speed
        self.kp = kp
        self.threshold = threshold
        self.logger = PathLogger()
        self.step_count = 0
        
        # Regulator parameters
        self.regulator_type = regulator_type
        self.regulator_kp = regulator_kp
        self.regulator_ki = regulator_ki
        self.regulator_kd = regulator_kd
        self.regulator = None
        
    def on_start(self, hardware_adapter):
        hardware_adapter.ev3.speaker.beep()
        print("\n=== Lane Keeping Started ===")
        print("Base speed:", self.base_speed, "mm/s")
        print("Proportional gain (Kp):", self.kp)
        print("Line threshold:", self.threshold)
        
        # Initialize regulator
        if self.regulator_type == "PID":
            self.regulator = PIDRegulator(kp=self.regulator_kp, ki=self.regulator_ki, 
                                         kd=self.regulator_kd, min_output=-100, max_output=100)
            print("Regulator: PID (Kp=", self.regulator_kp, ", Ki=", self.regulator_ki, ", Kd=", self.regulator_kd, ")")
        elif self.regulator_type == "PIFuzzy":
            self.regulator = PIFuzzyRegulator(kp_base=self.regulator_kp, ki_base=self.regulator_ki, 
                                             min_output=-100, max_output=100)
            print("Regulator: PI-Fuzzy (Kp_base=", self.regulator_kp, ", Ki_base=", self.regulator_ki, ")")
        else:
            self.regulator = None
            print("Regulator: None (using simple proportional control)")
        
        print("="*30 + "\n")
        
    def step(self, hardware_adapter, dt):
        self.step_count += 1
        left_r = hardware_adapter.left_reflection()
        right_r = hardware_adapter.right_reflection()
        
        # Log sensor readings periodically (every 50 steps to avoid spam)
        if self.step_count % 50 == 0:
            self.logger.log_sensor_reading("left_color", left_r)
            self.logger.log_sensor_reading("right_color", right_r)
        
        left_on_line = left_r < self.threshold
        right_on_line = right_r < self.threshold
        
        if left_on_line and right_on_line:
            hardware_adapter.stop()
            hardware_adapter.ev3.speaker.beep()
            self.logger.log_decision("FINISH LINE DETECTED", "Both sensors on line")
            return
        
        error = right_r - left_r
        turn_rate = self.kp * error
        
        if turn_rate > 200:
            turn_rate = 200
        if turn_rate < -200:
            turn_rate = -200
        
        hardware_adapter.drive_turn_rate(self.base_speed, turn_rate)
        
    def on_stop(self, hardware_adapter):
        hardware_adapter.stop()
        print("\n" + "="*40)
        print("=== Lane Keeping Stopped ===")
        summary = self.logger.get_summary()
        print("Total steps:", self.step_count)
        print("Logged events:", summary['total_actions'])
        print("="*40 + "\n")
        
        # Export data to files in current directory
        print("\nExporting data to files...")
        DataExporter.export_log_to_file(self.logger, "lane_log.txt")
        DataExporter.export_training_data(self.logger, "lane_training.csv")