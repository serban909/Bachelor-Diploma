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
    def __init__(self, distance_threshold_low=35, distance_threshold_high=50, base_speed=30, min_forward_distance=30,
                 regulator_type="PID", regulator_kp=2.0, regulator_ki=0.01, regulator_kd=0.02):
        """
        Continuous wall-following maze solver using threshold band approach.
        
        Args:
            distance_threshold_low: Minimum acceptable distance from left wall (mm)
            distance_threshold_high: Maximum acceptable distance from left wall (mm)
            base_speed: Base forward speed (mm/s) - moderate for safety
            min_forward_distance: Minimum distance before obstacle avoidance (mm)
            regulator_type: "PID" or "PIFuzzy"
            regulator_kp: Proportional gain for wall following
            regulator_ki: Integral gain for wall following
            regulator_kd: Derivative gain for wall following
        """
        self.distance_threshold_low = distance_threshold_low
        self.distance_threshold_high = distance_threshold_high
        self.target_wall_distance = (distance_threshold_low + distance_threshold_high) / 2.0  # Center of band
        self.base_speed = base_speed
        self.min_forward_distance = min_forward_distance
        
        # Use logger for tracking
        self.logger = PathLogger()
        self.step_count = 0
        self.log_interval = 100  # Log every 100 steps
        
        # Regulator parameters
        self.regulator_type = regulator_type
        self.regulator_kp = regulator_kp
        self.regulator_ki = regulator_ki
        self.regulator_kd = regulator_kd
        self.regulator = None
        
        # State for continuous operation
        self.total_distance_traveled = 0
        
    def on_start(self, hardware_adapter):
        hardware_adapter.ev3.speaker.beep()
        print("\n=== Continuous Wall-Following Maze Solver ===")
        print("Wall distance band:", self.distance_threshold_low, "-", self.distance_threshold_high, "mm")
        print("Target (center):", int(self.target_wall_distance), "mm")
        print("Base speed:", self.base_speed, "mm/s")
        print("Minimum forward distance:", self.min_forward_distance, "mm")
        
        # Initialize regulator for wall following
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
            print("Regulator: None (not recommended for continuous wall following)")
        
        print("Mode: Continuous left-hand wall following")
        print("="*30 + "\n")
        
        self.logger.log_decision("START", "Continuous wall-following mode initialized")
        
    def step(self, hardware_adapter, dt):
        """
        Continuous wall-following control loop.
        Uses PID/PI-Fuzzy regulator to maintain constant distance from left wall.
        """
        self.step_count += 1
        
        # Get sensor readings from SEPARATE sensors
        forward_distance = hardware_adapter.distance_mm()  # Forward-facing sensor (Port S4)
        left_wall_distance = hardware_adapter.left_wall_distance_mm()  # Left-facing sensor (Port S1)
        
        # Log sensor data periodically
        if self.step_count % self.log_interval == 0:
            self.logger.log_sensor_reading("left_wall", left_wall_distance, "")
            self.logger.log_sensor_reading("forward", forward_distance, "")
            print("Step", self.step_count, "| Left wall:", left_wall_distance, "mm | Forward:", forward_distance, "mm")
        
        # Check if obstacle ahead (emergency stop/turn)
        if forward_distance < self.min_forward_distance:
            # Obstacle detected ahead - perform left-hand rule turn
            self._handle_obstacle(hardware_adapter, forward_distance)
            return
        
        # Threshold band wall following for better curve handling
        steering = 0
        
        if left_wall_distance < self.distance_threshold_low:
            # TOO CLOSE to wall - steer RIGHT (away from wall)
            if self.regulator is not None:
                # Use PID with virtual target at high threshold
                steering = self.regulator.compute(self.distance_threshold_high, left_wall_distance, dt)
            else:
                # Simple proportional: closer = more right steering
                error = left_wall_distance - self.distance_threshold_low
                steering = -error * 3.0  # Negative error, so negative steering = right turn
                
        elif left_wall_distance > self.distance_threshold_high:
            # TOO FAR from wall - steer LEFT (toward wall)
            if self.regulator is not None:
                # Use PID with virtual target at low threshold
                steering = self.regulator.compute(self.distance_threshold_low, left_wall_distance, dt)
            else:
                # Simple proportional: farther = more left steering
                error = left_wall_distance - self.distance_threshold_high
                steering = -error * 3.0  # Positive error, negative steering = left turn
                
        else:
            # WITHIN BAND - go straight, no correction needed!
            steering = 0
            if self.regulator is not None:
                self.regulator.reset()  # Reset PID state to prevent integral windup
        
        # Apply steering limits
        if steering > 300:
            steering = 300
        elif steering < -300:
            steering = -300
        
        # Drive forward with calculated steering
        hardware_adapter.drive_turn_rate(self.base_speed, steering)
        
        # Track distance (approximate)
        distance_this_step = self.base_speed * dt
        self.total_distance_traveled += distance_this_step
    
    def _handle_obstacle(self, hardware_adapter, forward_distance):
        """
        Handle obstacle detection using left-hand rule.
        This is called when forward path is blocked.
        """
        hardware_adapter.stop()
        self.logger.log_decision("OBSTACLE AHEAD", "Distance: " + str(forward_distance) + "mm")
        print("\n*** OBSTACLE DETECTED - Applying left-hand rule ***")
        
        # Try turning left first (left-hand rule)
        print("Attempting LEFT turn...")
        hardware_adapter.turn_angle(-90)
        self.logger.log_turn(-90, "Left-hand rule: turn left")
        wait(300)
        
        # Check if path is clear after turning
        new_forward_distance = hardware_adapter.distance_mm()
        self.logger.log_sensor_reading("forward_after_left_turn", new_forward_distance, "")
        
        if new_forward_distance >= self.min_forward_distance:
            # Path is clear, continue
            self.logger.log_decision("LEFT TURN SUCCESS", "Path clear after left turn")
            print("Left turn successful, continuing...")
            if self.regulator is not None:
                self.regulator.reset()  # Reset PID state after discrete turn
            return
        
        # Left didn't work, try going back and turning right
        print("Left blocked, trying RIGHT...")
        hardware_adapter.turn_angle(180)  # Turn 180 from left = 90 right from original
        self.logger.log_turn(180, "Left blocked, turning right instead")
        wait(300)
        
        new_forward_distance = hardware_adapter.distance_mm()
        self.logger.log_sensor_reading("forward_after_right_turn", new_forward_distance, "")
        
        if new_forward_distance >= self.min_forward_distance:
            self.logger.log_decision("RIGHT TURN SUCCESS", "Path clear after right turn")
            print("Right turn successful, continuing...")
            if self.regulator is not None:
                self.regulator.reset()
            return
        
        # Both left and right blocked - turn around 180
        print("Dead end detected, turning around...")
        hardware_adapter.turn_angle(180)
        self.logger.log_turn(180, "Dead end - turning around")
        self.logger.log_decision("DEAD END", "Turned around 180 degrees")
        wait(300)
        
        if self.regulator is not None:
            self.regulator.reset()
        
    def on_stop(self, hardware_adapter):
        hardware_adapter.stop()
        print("\n" + "="*40)
        print("=== Continuous Wall-Following Maze Solver Stopped ===")
        summary = self.logger.get_summary()
        print("Total steps executed:", self.step_count)
        print("Approximate distance traveled:", int(self.total_distance_traveled), "mm")
        print("Total decisions made:", summary['total_decisions'])
        print("Total actions logged:", summary['total_actions'])
        
        # Show regulator final state
        if self.regulator is not None:
            state = self.regulator.get_state()
            print("\nRegulator final state:")
            if self.regulator_type == "PID":
                print("  Kp:", state['kp'], "Ki:", state['ki'], "Kd:", state['kd'])
                print("  Integral term:", round(state['integral'], 2))
                print("  Last error:", round(state['previous_error'], 2))
            elif self.regulator_type == "PIFuzzy":
                print("  Kp_base:", state['kp_base'], "Ki_base:", state['ki_base'])
                print("  Integral term:", round(state['integral'], 2))
        
        print("="*40 + "\n")
        
        # Print summary of major events
        print("\n=== MAJOR EVENTS LOG ===")
        for entry in self.logger.get_log_entries():
            if entry['type'] == 'decision':
                msg = str(entry['number']) + ". " + entry['decision']
                if entry['detail']:
                    msg += " (" + entry['detail'] + ")"
                print(msg)
            elif entry['type'] == 'turn':
                print("  -> TURN: " + str(entry['angle']) + " degrees - " + entry['description'])
        
        # Export data to files
        print("\n" + "="*40)
        print("Exporting data to files...")
        DataExporter.export_log_to_file(self.logger, "maze_run_log.txt")
        DataExporter.export_training_data(self.logger, "maze_training.csv")
        print("Data exported successfully!")
        print("="*40)