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

class MazeSolver(Behavior):
    def __init__(self, wall_distance_mm=180, forward_distance_mm=150, probe_turn_deg=90, probe_pause_ms=300,
                 regulator_type="PID", regulator_kp=2.0, regulator_ki=0.4, regulator_kd=0.1):
        self.wall_distance = wall_distance_mm
        self.forward_distance = forward_distance_mm
        self.probe_turn = probe_turn_deg
        self.probe_pause = probe_pause_ms
        self.state = 'DECIDING'
        
        # Use logger for tracking
        self.logger = PathLogger()
        
        # Regulator parameters
        self.regulator_type = regulator_type
        self.regulator_kp = regulator_kp
        self.regulator_ki = regulator_ki
        self.regulator_kd = regulator_kd
        self.regulator = None
        
    def on_start(self, hardware_adapter):
        hardware_adapter.ev3.speaker.beep()
        print("\n=== Maze Solver Started ===")
        print("Wall distance threshold:", self.wall_distance, "mm")
        print("Forward step distance:", self.forward_distance, "mm")
        
        # Initialize regulator
        if self.regulator_type == "PID":
            self.regulator = PIDRegulator(kp=self.regulator_kp, ki=self.regulator_ki, 
                                         kd=self.regulator_kd, min_output=-200, max_output=200)
            print("Regulator: PID (Kp=", self.regulator_kp, ", Ki=", self.regulator_ki, ", Kd=", self.regulator_kd, ")")
        elif self.regulator_type == "PIFuzzy":
            self.regulator = PIFuzzyRegulator(kp_base=self.regulator_kp, ki_base=self.regulator_ki, 
                                             min_output=-200, max_output=200)
            print("Regulator: PI-Fuzzy (Kp_base=", self.regulator_kp, ", Ki_base=", self.regulator_ki, ")")
        else:
            self.regulator = None
            print("Regulator: None (using direct motor control)")
        
        print("="*30 + "\n")
        
    def _is_blocked(self, hardware_adapter):
        distance = hardware_adapter.distance_mm()
        blocked = distance < self.wall_distance
        interpretation = "BLOCKED" if blocked else "FREE"
        self.logger.log_sensor_reading("ultrasonic", distance, interpretation)
        return blocked
    
    def step(self, hardware_adapter, dt):
        if self.state == 'DECIDING':
            summary = self.logger.get_summary()
            print("\n--- Decision", summary['total_decisions'] + 1, "---")
            
            # ALWAYS CHECK FORWARD FIRST
            print("Checking FORWARD...")
            self.logger.log_decision("Check FORWARD", "Priority check")
            forward_blocked = self._is_blocked(hardware_adapter)
            
            if not forward_blocked:
                # Forward is clear - just go straight! (most common case)
                self.logger.log_decision("FORWARD clear", "Going straight")
                hardware_adapter.straight(self.forward_distance)
                self.logger.log_move(self.forward_distance)
                self.state = 'DECIDING'
                return
            
            # Forward is blocked - now apply left-hand rule
            self.logger.log_decision("FORWARD blocked", "Applying left-hand rule")
            
            # Check left (turn left, check, turn back if blocked)
            print("Checking LEFT...")
            self.logger.log_decision("Check LEFT", "Turning -90 degrees")
            hardware_adapter.turn_angle(-self.probe_turn)
            self.logger.log_turn(-self.probe_turn, "Turn left to check")
            wait(self.probe_pause)
            
            left_blocked = self._is_blocked(hardware_adapter)
            
            if not left_blocked:
                # Left is free, take it
                self.logger.log_decision("Take LEFT path", "Path is free")
                hardware_adapter.straight(self.forward_distance)
                self.logger.log_move(self.forward_distance)
                self.state = 'DECIDING'
                return
            else:
                # Left is blocked, turn back to original direction
                self.logger.log_decision("LEFT blocked", "Returning to center")
                hardware_adapter.turn_angle(self.probe_turn)
                self.logger.log_turn(self.probe_turn, "Turn right back to center")
                wait(self.probe_pause)
            
            # Check right (turn right, check, turn back if blocked)
            print("Checking RIGHT...")
            self.logger.log_decision("Check RIGHT", "Turning +90 degrees")
            hardware_adapter.turn_angle(self.probe_turn)
            self.logger.log_turn(self.probe_turn, "Turn right to check")
            wait(self.probe_pause)
            
            right_blocked = self._is_blocked(hardware_adapter)
            
            if not right_blocked:
                # Right is free, take it
                self.logger.log_decision("Take RIGHT path", "Path is free")
                hardware_adapter.straight(self.forward_distance)
                self.logger.log_move(self.forward_distance)
                self.state = 'DECIDING'
                return
            else:
                # All paths blocked - turn around 180
                self.logger.log_decision("ALL paths blocked", "Dead end - turning around")
                hardware_adapter.turn_angle(-self.probe_turn)
                self.logger.log_turn(-self.probe_turn, "Turn left back to center")
                wait(self.probe_pause)
                hardware_adapter.turn_angle(180)
                self.logger.log_turn(180, "Turn around 180 degrees")
                wait(self.probe_pause)
                # Don't move forward here, let next iteration decide
                self.state = 'DECIDING'
                return
        
    def on_stop(self, hardware_adapter):
        hardware_adapter.stop()
        print("\n" + "="*40)
        print("=== Maze Solver Stopped ===")
        summary = self.logger.get_summary()
        print("Total distance traveled:", summary['total_distance'], "mm")
        print("Total decisions made:", summary['total_decisions'])
        print("="*40 + "\n")
        
        # Print complete log to console
        print("\n=== COMPLETE LOG ===")
        for entry in self.logger.get_log_entries():
            if entry['type'] == 'decision':
                msg = str(entry['number']) + ". DECISION: " + entry['decision']
                if entry['detail']:
                    msg += " (" + entry['detail'] + ")"
                print(msg)
            elif entry['type'] == 'turn':
                print("  TURN: " + str(entry['angle']) + " degrees - " + entry['description'])
            elif entry['type'] == 'move':
                print("  MOVE: " + str(entry['distance']) + "mm")
        
        # Export data to files
        print("\n" + "="*40)
        print("Exporting data to files...")
        DataExporter.export_log_to_file(self.logger, "maze_run_log.txt")
        DataExporter.export_training_data(self.logger, None, "maze_training.csv")
        print("="*40)