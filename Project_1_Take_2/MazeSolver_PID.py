#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

LEFT_MOTOR_PORT = Port.A
RIGHT_MOTOR_PORT = Port.C
FORWARD_ULTRASONIC_PORT = Port.S4  
LEFT_ULTRASONIC_PORT = Port.S1     

WHEEL_DIAMETER = 55.5  
AXLE_TRACK = 104       

DISTANCE_THRESHOLD_LOW = 30   
DISTANCE_THRESHOLD_HIGH = 50  
TARGET_WALL_DISTANCE = (DISTANCE_THRESHOLD_LOW + DISTANCE_THRESHOLD_HIGH) / 2.0  

BASE_SPEED = 25                
SLOW_SPEED = 12.5              


MIN_FORWARD_DISTANCE = 60      
OBSTACLE_THRESHOLD = 30       
OPEN_PASSAGE_THRESHOLD = 60    

KP = 6.0   
KI = 0.02  
KD = 0.05  

PID_MIN_OUTPUT = -100  
PID_MAX_OUTPUT = 100   
INTEGRAL_MIN = -1000.0 
INTEGRAL_MAX = 1000.0  

PEEK_ANGLE = 30       

LOG_INTERVAL = 100     

ev3 = EV3Brick()

left_motor = Motor(LEFT_MOTOR_PORT)
right_motor = Motor(RIGHT_MOTOR_PORT)

robot = DriveBase(left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)

forward_sensor = UltrasonicSensor(FORWARD_ULTRASONIC_PORT) 
left_sensor = UltrasonicSensor(LEFT_ULTRASONIC_PORT)        

pid_integral = 0.0
pid_previous_error = 0.0

forward_history = [0, 0, 0] 
left_history = [0, 0, 0]

step_count = 0
total_distance = 0

def filter_sensor_reading(new_reading, history):
    history[0] = history[1]
    history[1] = history[2]
    history[2] = new_reading
    
    return (history[0] + history[1] + history[2]) / 3.0

def read_sensors():
    global forward_history, left_history
    
    forward_raw = forward_sensor.distance()
    left_raw = left_sensor.distance()
    
    forward_filtered = filter_sensor_reading(forward_raw, forward_history)
    left_filtered = filter_sensor_reading(left_raw, left_history)
    
    return forward_filtered, left_filtered

def pid_compute(setpoint, measured_value, dt):
    global pid_integral, pid_previous_error
    
    error = setpoint - measured_value
    
    p_term = KP * error
    
    if dt > 0:
        pid_integral += error * dt
        if pid_integral > INTEGRAL_MAX:
            pid_integral = INTEGRAL_MAX
        elif pid_integral < INTEGRAL_MIN:
            pid_integral = INTEGRAL_MIN
    i_term = KI * pid_integral
    
    if dt > 0:
        derivative = (error - pid_previous_error) / dt
    else:
        derivative = 0.0
    d_term = KD * derivative
    
    output = p_term + i_term + d_term
    
    if output > PID_MAX_OUTPUT:
        output = PID_MAX_OUTPUT
    elif output < PID_MIN_OUTPUT:
        output = PID_MIN_OUTPUT
    
    pid_previous_error = error
    
    return output


def pid_reset():
    global pid_integral, pid_previous_error
    pid_integral = 0.0
    pid_previous_error = 0.0


def peek_and_turn(target_angle, reason):
    print("PEEK-AND-TURN: " + reason)
    print("  Target: " + str(target_angle) + " degrees")
    
    step = PEEK_ANGLE if target_angle > 0 else -PEEK_ANGLE
    total_turned = 0
    turns_made = 0
    
    while abs(total_turned) < abs(target_angle):
        robot.turn(step)
        total_turned += step
        turns_made += 1
        
        print("  Peek " + str(turns_made) + ": " + str(abs(total_turned)) + " degrees turned")
        
        forward_dist, left_dist = read_sensors()
        print("    Forward=" + str(int(forward_dist)) + "mm, Left=" + str(int(left_dist)) + "mm")
        
        forward_clear = forward_dist > MIN_FORWARD_DISTANCE
        left_clear = left_dist > OPEN_PASSAGE_THRESHOLD
        
        if forward_clear and left_clear:
            print("  PATH CLEAR after " + str(abs(total_turned)) + " degrees!")
            break
        elif abs(total_turned) >= abs(target_angle):
            print("  Completed full " + str(abs(target_angle)) + " degree turn")
            break
    
    return total_turned


def handle_obstacle(forward_distance, left_distance):
    print("\n=== OBSTACLE DETECTED ===")
    print("Forward: " + str(int(forward_distance)) + "mm")
    print("Left: " + str(int(left_distance)) + "mm")
    
    robot.stop()
    wait(200)
    
    pid_reset()
    
    forward_distance, left_distance = read_sensors()

    if left_distance > OPEN_PASSAGE_THRESHOLD:
        print("Decision: Left path CLEAR - turning left")
        angle_turned = peek_and_turn(-90, "Left passage detected")
        print("Turned " + str(abs(angle_turned)) + " degrees left\n")
        return
    
    if forward_distance > MIN_FORWARD_DISTANCE:
        print("Decision: Forward CLEAR - continuing straight")
        print("(False alarm or edge case)\n")
        return
    
    print("Decision: Front AND Left blocked - trying RIGHT")
    robot.turn(90)  # Turn right 90 degrees (positive = right)
    print("Turned 90 degrees right")
    
    forward_distance, left_distance = read_sensors()
    if forward_distance <= OBSTACLE_THRESHOLD:
        print("Still blocked! Turning around...")
        robot.turn(90)  # Another 90 right = 180 total from original heading
        print("Turned 180 degrees total (turnaround)\n")
    else:
        print("Right turn successful!\n")


def handle_open_left_passage(left_distance):
    print("\n=== OPEN LEFT PASSAGE ===")
    print("Left wall: " + str(int(left_distance)) + "mm")
    
    robot.stop()
    wait(200)
    
    pid_reset()
    
    print("Decision: Taking left passage")
    angle_turned = peek_and_turn(-90, "Open left passage")
    print("Turned " + str(abs(angle_turned)) + " degrees into passage\n")

def main():
    global step_count, total_distance
    
    ev3.speaker.beep()
    print("\n" + "="*50)
    print("MAZE SOLVER - PID WALL FOLLOWING")
    print("="*50)
    print("Configuration:")
    print("  Target wall distance: " + str(TARGET_WALL_DISTANCE) + "mm")
    print("  Base speed: " + str(BASE_SPEED) + "mm/s")
    print("  PID gains: Kp=" + str(KP) + ", Ki=" + str(KI) + ", Kd=" + str(KD))
    print("  Forward sensor: Port S4")
    print("  Left sensor: Port S1")
    print("="*50 + "\n")
    
    wait(1000)
    print("Starting in 3...")
    wait(1000)
    print("2...")
    wait(1000)
    print("1...\n")
    
    last_time = 0
    
    try:
        while True:
            step_count += 1
            
            current_time = step_count * 0.02  
            dt = current_time - last_time
            last_time = current_time
            
            forward_distance, left_wall_distance = read_sensors()
            
            if step_count % LOG_INTERVAL == 0:
                print("Step " + str(step_count) + " | Left wall: " + str(int(left_wall_distance)) + 
                      " mm | Forward: " + str(int(forward_distance)) + " mm")

            if forward_distance <= OBSTACLE_THRESHOLD:
                handle_obstacle(forward_distance, left_wall_distance)
                continue
            
            if left_wall_distance > OPEN_PASSAGE_THRESHOLD:
                handle_open_left_passage(left_wall_distance)
                continue
            
            if forward_distance <= MIN_FORWARD_DISTANCE:
                speed = SLOW_SPEED
                if step_count % LOG_INTERVAL == 0:
                    print("  -> Slowing down (obstacle at " + str(int(forward_distance)) + "mm)")
            else:
                speed = BASE_SPEED

            steering = pid_compute(TARGET_WALL_DISTANCE, left_wall_distance, dt)
            
            if step_count % LOG_INTERVAL == 0:
                print("  -> Steering: " + str(int(steering)) + " deg/s")

            robot.drive(speed, steering)

            wait(20)
    
    except KeyboardInterrupt:
        robot.stop()
        print("\n" + "="*50)
        print("MAZE SOLVER STOPPED")
        print("Total steps: " + str(step_count))
        print("="*50 + "\n")
        ev3.speaker.beep()

if __name__ == "__main__":
    main()
