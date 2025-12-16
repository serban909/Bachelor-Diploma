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
    def __init__(self, distance_threshold_low, distance_threshold_high, base_speed, min_forward_distance,
                 obstacle_threshold, regulator_type, regulator_kp, regulator_ki, regulator_kd):
        """
        Continuous wall-following maze solver using threshold band approach.
        
        All parameters are REQUIRED - configure them in main.py.
        
        Args:
            distance_threshold_low: Minimum acceptable distance from left wall (mm)
            distance_threshold_high: Maximum acceptable distance from left wall (mm)
            base_speed: Base forward speed (mm/s)
            min_forward_distance: Distance to slow down and prepare for turn (mm)
            obstacle_threshold: Distance to actually stop and turn (mm)
            regulator_type: "PID" or "PIFuzzy"
            regulator_kp: Proportional gain for wall following
            regulator_ki: Integral gain for wall following
            regulator_kd: Derivative gain for wall following
        """
        self.distance_threshold_low = distance_threshold_low
        self.distance_threshold_high = distance_threshold_high
        self.target_wall_distance = (distance_threshold_low + distance_threshold_high) / 2.0  # Center of band
        self.base_speed = base_speed
        self.min_forward_distance = min_forward_distance  # Warning distance
        self.obstacle_threshold = obstacle_threshold  # Critical distance for turning
        
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
        
        # Turn management to prevent false positives
        self.steps_since_last_turn = 0
        self.turn_cooldown_steps = 50  # Wait 50 steps after turn before checking left passages
        self.open_passage_consecutive_count = 0
        self.open_passage_required_count = 3  # Require 3 consecutive readings before turning
        
    def on_start(self, hardware_adapter):
        hardware_adapter.ev3.speaker.beep()
        print("\n=== Continuous Wall-Following Maze Solver ===")
        print("Wall distance band:", self.distance_threshold_low, "-", self.distance_threshold_high, "mm")
        print("Target (center):", int(self.target_wall_distance), "mm")
        print("Base speed:", self.base_speed, "mm/s")
        print("Approach distance (slow down):", self.min_forward_distance, "mm")
        print("Obstacle threshold (turn):", self.obstacle_threshold, "mm")
        
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
        Optimized decision-making using BOTH ultrasonic sensors simultaneously.
        
        Decision Tree:
        1. CRITICAL OBSTACLE: Front <= 35mm OR Left <= 35mm → STOP and turn
        2. APPROACHING OBSTACLE: 35mm < Front <= 60mm → SLOW DOWN, continue forward
        3. BOTH FREE: Front > 60mm AND Left > 60mm → GO FORWARD (not turn)
        4. NORMAL WALL-FOLLOWING: Front > 60mm AND 30mm < Left <= 60mm → PID control
        """
        self.step_count += 1
        self.steps_since_last_turn += 1
        
        # Read BOTH sensors simultaneously
        forward_distance = hardware_adapter.distance_mm()  # Forward-facing sensor (Port S4)
        left_wall_distance = hardware_adapter.left_wall_distance_mm()  # Left-facing sensor (Port S1)
        
        # Log BOTH sensor readings every cycle
        if self.step_count % self.log_interval == 0:
            self.logger.log_sensor_reading(
                "dual_sensors",
                "Forward=" + str(forward_distance) + "mm, Left=" + str(left_wall_distance) + "mm",
                "Step " + str(self.step_count)
            )
            print("Step", self.step_count, "| Forward:", forward_distance, "mm | Left:", left_wall_distance, "mm")
        
        # DECISION 1: CRITICAL OBSTACLE - Must turn NOW (30-35mm threshold)
        if forward_distance <= self.obstacle_threshold or left_wall_distance <= self.obstacle_threshold:
            hardware_adapter.stop()
            self.logger.log_decision(
                "CRITICAL_OBSTACLE",
                "Forward=" + str(forward_distance) + "mm, Left=" + str(left_wall_distance) + "mm (<=" + str(self.obstacle_threshold) + "mm)"
            )
            print("\n*** CRITICAL OBSTACLE - TURNING ***")
            print("Forward:", forward_distance, "mm | Left:", left_wall_distance, "mm")
            
            self._handle_obstacle_with_sensors(hardware_adapter, forward_distance, left_wall_distance)
            self.steps_since_last_turn = 0
            self.open_passage_consecutive_count = 0
            return
        
        # DECISION 2: APPROACHING OBSTACLE - Slow down but continue (35mm < Front <= 60mm)
        if forward_distance <= self.min_forward_distance:
            # Reduce speed for better control
            reduced_speed = self.base_speed * 0.5  # 50% speed
            
            # Still follow left wall if present
            if left_wall_distance < self.distance_threshold_low:
                # Too close to left wall - steer right
                if self.regulator is not None:
                    steering = self.regulator.compute(self.target_wall_distance, left_wall_distance, dt)
                else:
                    steering = (self.target_wall_distance - left_wall_distance) * 2.0
                
                if steering > 200:
                    steering = 200
                elif steering < -200:
                    steering = -200
                    
                hardware_adapter.drive_turn_rate(reduced_speed, steering)
            else:
                # Go forward slowly
                hardware_adapter.drive_forward(reduced_speed)
            
            distance_this_step = reduced_speed * dt
            self.total_distance_traveled += distance_this_step
            return
        
        # DECISION 3: BOTH SENSORS FREE - Go forward, don't turn left yet
        if left_wall_distance > self.min_forward_distance:
            # No left wall detected - but keep going forward
            # Only turn left if we've seen it consistently and cooldown passed
            
            if self.steps_since_last_turn > self.turn_cooldown_steps:
                self.open_passage_consecutive_count += 1
                
                if self.open_passage_consecutive_count >= self.open_passage_required_count:
                    # Confirmed open left passage - turn left now
                    hardware_adapter.stop()
                    self.logger.log_decision(
                        "OPEN_LEFT_PASSAGE",
                        "Forward=" + str(forward_distance) + "mm, Left=" + str(left_wall_distance) + "mm (open)"
                    )
                    self._handle_open_left_passage(hardware_adapter, left_wall_distance)
                    self.steps_since_last_turn = 0
                    self.open_passage_consecutive_count = 0
                    return
            
            # Continue forward while counting
            hardware_adapter.drive_forward(self.base_speed)
            distance_this_step = self.base_speed * dt
            self.total_distance_traveled += distance_this_step
            return
        
        # DECISION 4: NORMAL WALL-FOLLOWING (Front > 60mm, Left detected)
        # Use threshold band control to stay centered
        self.open_passage_consecutive_count = 0  # Reset since wall is present
        
        if left_wall_distance < self.distance_threshold_low:
            # TOO CLOSE to left wall - steer RIGHT
            if self.regulator is not None:
                steering = self.regulator.compute(self.target_wall_distance, left_wall_distance, dt)
            else:
                steering = (self.target_wall_distance - left_wall_distance) * 3.0
            
            if steering > 300:
                steering = 300
            elif steering < -300:
                steering = -300
            
            hardware_adapter.drive_turn_rate(self.base_speed, steering)
            
        elif left_wall_distance > self.distance_threshold_high:
            # TOO FAR from left wall - steer LEFT
            if self.regulator is not None:
                steering = self.regulator.compute(self.target_wall_distance, left_wall_distance, dt)
            else:
                steering = (self.target_wall_distance - left_wall_distance) * 3.0
            
            if steering > 300:
                steering = 300
            elif steering < -300:
                steering = -300
            
            hardware_adapter.drive_turn_rate(self.base_speed, steering)
            
        else:
            # WITHIN BAND - go straight
            if self.regulator is not None:
                self.regulator.reset()
            hardware_adapter.drive_forward(self.base_speed)
        
        distance_this_step = self.base_speed * dt
        self.total_distance_traveled += distance_this_step
    
    def _handle_obstacle_with_sensors(self, hardware_adapter, forward_distance, left_wall_distance):
        """
        Handle obstacle using BOTH sensor readings for intelligent navigation.
        
        Logic:
        - Front blocked + Left clear (<500mm threshold) → Turn LEFT only
        - Front blocked + Left blocked (>=500mm) → Try RIGHT
          - Right clear → Continue
          - Right blocked → Turn 180° (dead end)
        
        Args:
            hardware_adapter: Hardware interface
            forward_distance: Current forward sensor reading (mm)
            left_wall_distance: Current left sensor reading (mm)
        """
        
        # Check left sensor status at time of obstacle detection
        # Left is clear if distance > obstacle_threshold (typically > 35mm means no immediate wall)
        if left_wall_distance > 100:  # No immediate left wall = left path likely clear
            # LEFT PATH CLEAR - Turn left immediately (no need to check after turn)
            print("Decision: Left path CLEAR, turning LEFT...")
            self.logger.log_decision(
                "TURN_LEFT_CLEAR",
                "Front blocked (" + str(forward_distance) + "mm), Left clear (" + str(left_wall_distance) + "mm)"
            )
            
            hardware_adapter.turn_angle(-90)
            self.logger.log_turn(-90, "Left path clear - turn left")
            wait(500)
            
            # Verify forward is now clear after turn
            new_forward = hardware_adapter.distance_mm()
            new_left = hardware_adapter.left_wall_distance_mm()
            self.logger.log_sensor_reading(
                "after_left_turn",
                "Forward=" + str(new_forward) + "mm, Left=" + str(new_left) + "mm",
                "Post-turn sensor check"
            )
            print("After LEFT turn: Forward=", new_forward, "mm, Left=", new_left, "mm")
            
            if new_forward >= self.min_forward_distance:
                self.logger.log_decision("LEFT_SUCCESS", "Path clear after left turn")
                print("Left turn successful, continuing...")
            else:
                self.logger.log_decision("LEFT_BLOCKED", "Warning: Still blocked after left turn")
                print("Warning: Path still blocked after left turn!")
            
            if self.regulator is not None:
                self.regulator.reset()
            return
        
        else:  # left_wall_distance < 500 (left wall present = left path blocked)
            # BOTH FRONT AND LEFT BLOCKED - Try turning RIGHT
            print("Decision: Front AND Left blocked, trying RIGHT...")
            self.logger.log_decision(
                "TURN_RIGHT_TRY",
                "Front blocked (" + str(forward_distance) + "mm), Left blocked (" + str(left_wall_distance) + "mm)"
            )
            
            hardware_adapter.turn_angle(90)
            self.logger.log_turn(90, "Front+Left blocked - try right")
            wait(500)
            
            # Check if right path is clear
            new_forward = hardware_adapter.distance_mm()
            new_left = hardware_adapter.left_wall_distance_mm()
            self.logger.log_sensor_reading(
                "after_right_turn",
                "Forward=" + str(new_forward) + "mm, Left=" + str(new_left) + "mm",
                "Post-turn sensor check"
            )
            print("After RIGHT turn: Forward=", new_forward, "mm, Left=", new_left, "mm")
            
            if new_forward >= self.min_forward_distance:
                # RIGHT PATH CLEAR - Continue
                self.logger.log_decision("RIGHT_SUCCESS", "Right path clear")
                print("Right turn successful, continuing...")
                
                if self.regulator is not None:
                    self.regulator.reset()
                return
            
            else:
                # ALL THREE DIRECTIONS BLOCKED - DEAD END
                print("Decision: DEAD END detected, turning 180°...")
                self.logger.log_decision(
                    "DEAD_END",
                    "All paths blocked - turning around"
                )
                
                hardware_adapter.turn_angle(180)
                self.logger.log_turn(180, "Dead end - turn around")
                wait(500)
                
                # Final sensor check
                final_forward = hardware_adapter.distance_mm()
                final_left = hardware_adapter.left_wall_distance_mm()
                self.logger.log_sensor_reading(
                    "after_180_turn",
                    "Forward=" + str(final_forward) + "mm, Left=" + str(final_left) + "mm",
                    "After 180° turn"
                )
                print("After 180° turn: Forward=", final_forward, "mm, Left=", final_left, "mm")
                
                if self.regulator is not None:
                    self.regulator.reset()
                return
    
    def _handle_open_left_passage(self, hardware_adapter, left_wall_distance):
        """
        Handle open left passage detection (LEFT-HAND RULE).
        When no wall on left, always turn left to follow left-hand rule.
        """
        hardware_adapter.stop()
        self.logger.log_decision("OPEN LEFT PASSAGE", "Distance: " + str(left_wall_distance) + "mm (no wall)")
        print("\n*** OPEN LEFT PASSAGE - Turning left (left-hand rule) ***")
        print("Cooldown satisfied:", self.steps_since_last_turn, "steps since last turn")
        print("Consecutive readings:", self.open_passage_consecutive_count)
        
        # Turn left into the open passage - EXACT 90 degrees
        print("Turning LEFT 90 degrees into open passage...")
        hardware_adapter.turn_angle(-90)
        self.logger.log_turn(-90, "Open left passage - turn left")
        wait(500)  # Increased delay for robot to stabilize after turn
        
        # Check if forward path is clear
        forward_distance = hardware_adapter.distance_mm()
        self.logger.log_sensor_reading("forward_after_left_passage", forward_distance, "")
        
        if forward_distance >= self.min_forward_distance:
            self.logger.log_decision("LEFT PASSAGE SUCCESS", "Entered left passage")
            print("Entered left passage successfully, continuing...")
        else:
            # Forward blocked after turning - this shouldn't happen often
            self.logger.log_decision("LEFT PASSAGE BLOCKED", "Obstacle after turning left")
            print("Warning: Obstacle detected after turning left")
        
        if self.regulator is not None:
            self.regulator.reset()  # Reset PID state after discrete turn
        
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