from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from Behavior import Behavior
from PathLogger import PathLogger
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
        
        # Sensor filtering - moving average of last 3 readings
        self.forward_history = [1000, 1000, 1000]  # Initialize with large values
        self.left_history = [1000, 1000, 1000]
        self.filter_size = 3
        
    def _filter_sensor_reading(self, new_reading, history):
        """
        Apply moving average filter to sensor reading.
        Reduces noise from ultrasonic sensors.
        
        Args:
            new_reading: New sensor value (mm)
            history: List of previous readings
            
        Returns:
            Filtered (averaged) reading
        """
        # Shift history and add new reading
        history[0] = history[1]
        history[1] = history[2]
        history[2] = new_reading
        
        # Return average of last 3 readings
        return (history[0] + history[1] + history[2]) / 3.0
    
    def _peek_and_turn(self, hardware_adapter, target_angle, reason):
        """
        Incrementally rotate and check sensors before committing to full turn.
        Rotates in small steps (30 degrees) and checks if path is clear.
        
        Args:
            hardware_adapter: Hardware interface
            target_angle: Total angle to turn (positive=right, negative=left)
            reason: Description for logging
            
        Returns:
            True if path became clear during rotation, False if blocked
        """
        increment = 30 if target_angle > 0 else -30  # 30 degree steps
        total_turned = 0
        
        print("Peek-and-turn:", "RIGHT" if target_angle > 0 else "LEFT", "target:", target_angle, "degrees")
        
        while abs(total_turned) < abs(target_angle):
            # Turn one increment
            hardware_adapter.turn_angle(increment)
            total_turned += increment
            wait(300)  # Stabilize
            
            # Check sensors after this increment
            forward = int(self._filter_sensor_reading(hardware_adapter.distance_mm(), self.forward_history))
            left = int(self._filter_sensor_reading(hardware_adapter.left_wall_distance_mm(), self.left_history))
            
            print("  Rotated", total_turned, "deg | Forward:", forward, "mm, Left:", left, "mm")
            
            # Check if path is now clear
            if forward >= self.min_forward_distance:
                print("  Path CLEAR after", total_turned, "degrees!")
                self.logger.log_turn(total_turned, reason + " (clear after " + str(total_turned) + " deg)")
                self.logger.log_sensor_reading(
                    "peek_result",
                    "Forward=" + str(forward) + "mm, Left=" + str(left) + "mm",
                    "Path clear at " + str(total_turned) + " deg"
                )
                return True
        
        # Completed full rotation
        print("  Completed full", target_angle, "degree turn")
        self.logger.log_turn(target_angle, reason + " (full turn)")
        
        # Final sensor check
        forward = int(self._filter_sensor_reading(hardware_adapter.distance_mm(), self.forward_history))
        left = int(self._filter_sensor_reading(hardware_adapter.left_wall_distance_mm(), self.left_history))
        self.logger.log_sensor_reading(
            "after_full_turn",
            "Forward=" + str(forward) + "mm, Left=" + str(left) + "mm",
            "After " + str(target_angle) + " deg turn"
        )
        
        return forward >= self.min_forward_distance
    
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
        
        # Read BOTH sensors and apply filtering
        forward_raw = hardware_adapter.distance_mm()  # Forward-facing sensor (Port S4)
        left_raw = hardware_adapter.left_wall_distance_mm()  # Left-facing sensor (Port S1)
        
        # Apply moving average filter to reduce noise
        forward_distance = int(self._filter_sensor_reading(forward_raw, self.forward_history))
        left_wall_distance = int(self._filter_sensor_reading(left_raw, self.left_history))
        
        # Log BOTH sensor readings every cycle (filtered values)
        if self.step_count % self.log_interval == 0:
            self.logger.log_sensor_reading(
                "dual_sensors",
                "Forward=" + str(forward_distance) + "mm, Left=" + str(left_wall_distance) + "mm (filtered)",
                "Step " + str(self.step_count)
            )
            print("Step", self.step_count, "| Forward:", forward_distance, "mm | Left:", left_wall_distance, "mm (filtered)")
        
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
        
        steering = 0  # Initialize steering value
        
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
            
            # Log steering value periodically
            if self.step_count % self.log_interval == 0:
                print("  -> Steering: RIGHT", int(steering), "deg/s (too close to left wall)")
            
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
            
            # Log steering value periodically
            if self.step_count % self.log_interval == 0:
                print("  -> Steering: LEFT", int(steering), "deg/s (too far from left wall)")
            
            hardware_adapter.drive_turn_rate(self.base_speed, steering)
            
        else:
            # WITHIN BAND - go straight
            if self.regulator is not None:
                self.regulator.reset()
            
            # Log steering value periodically
            if self.step_count % self.log_interval == 0:
                print("  -> Steering: STRAIGHT (within band)")
            
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
        # Left is clear if distance > 60mm (lowered from 100mm for better detection)
        if left_wall_distance > 30:  # No immediate left wall = left path likely clear
            # LEFT PATH CLEAR - Peek and turn left incrementally
            print("Decision: Left path CLEAR, peek-and-turn LEFT...")
            self.logger.log_decision(
                "TURN_LEFT_CLEAR",
                "Front blocked (" + str(forward_distance) + "mm), Left clear (" + str(left_wall_distance) + "mm)"
            )
            
            # Use incremental rotation with sensor checks
            path_clear = self._peek_and_turn(hardware_adapter, -90, "Left path clear")
            
            if path_clear:
                self.logger.log_decision("LEFT_SUCCESS", "Path clear after left turn")
                print("Left turn successful, path is clear!")
            else:
                self.logger.log_decision("LEFT_BLOCKED", "Path still blocked after left turn")
                print("Warning: Path still blocked after left turn!")
            
            if self.regulator is not None:
                self.regulator.reset()
            return
        
        else:  # left_wall_distance <= 60mm (left wall present = left path blocked)
            # BOTH FRONT AND LEFT BLOCKED - Peek and try turning RIGHT
            print("Decision: Front AND Left blocked, peek-and-turn RIGHT...")
            self.logger.log_decision(
                "TURN_RIGHT_TRY",
                "Front blocked (" + str(forward_distance) + "mm), Left blocked (" + str(left_wall_distance) + "mm)"
            )
            
            # Use incremental rotation with sensor checks
            path_clear = self._peek_and_turn(hardware_adapter, 90, "Front+Left blocked - try right")
            
            if path_clear:
                # RIGHT PATH CLEAR - Continue
                self.logger.log_decision("RIGHT_SUCCESS", "Right path clear")
                print("Right turn successful, path is clear!")
                
                if self.regulator is not None:
                    self.regulator.reset()
                return
            
            else:
                # ALL THREE DIRECTIONS BLOCKED - DEAD END - Turn 180° incrementally
                print("Decision: DEAD END detected, peek-and-turn 180°...")
                self.logger.log_decision(
                    "DEAD_END",
                    "All paths blocked - turning around"
                )
                
                # Turn 180 degrees incrementally (will turn right another 90)
                self._peek_and_turn(hardware_adapter, 90, "Dead end - turn around (180 total)")
                
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
        
        # Peek and turn left into the open passage - incremental 30° steps
        print("Peek-and-turn LEFT into open passage...")
        path_clear = self._peek_and_turn(hardware_adapter, -90, "Open left passage - turn left")
        
        if path_clear:
            self.logger.log_decision("LEFT PASSAGE SUCCESS", "Entered left passage")
            print("Entered left passage successfully, path is clear!")
        else:
            # Forward blocked after turning - might need to continue or turn more
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
        
        print("="*40)