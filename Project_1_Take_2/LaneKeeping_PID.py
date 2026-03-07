#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
import socket

LEFT_MOTOR_PORT = Port.A
RIGHT_MOTOR_PORT = Port.C
LEFT_COLOR_PORT = Port.S2
RIGHT_COLOR_PORT = Port.S3

WHEEL_DIAMETER = 55.5  
AXLE_TRACK = 104       

# IMPORTANT: Run TestColor.py first to calibrate these values for your track!
# This code uses COMPARATIVE control with PID feedback
BLACK_THRESHOLD = 12    # Absolute black line detection (for stop condition)
CROSSING_THRESHOLD = 11 # Emergency: sensor is crossing/on black line
MIN_DIFFERENCE = 3      # Minimum sensor difference to trigger PID steering
BASE_SPEED = 50         # Base forward speed in mm/s when going straight
TURNING_SPEED = 30      # Reduced speed when steering (to prevent overshoot)
EMERGENCY_SPEED = 20    # Very slow when recovering from line crossing
EMERGENCY_TURN = 200    # Strong turn rate when crossing a line
STOP_CONFIRM_COUNT = 3  # Number of consecutive readings needed to confirm stop      

# PID tuning parameters for comparative control
KP = 8.0    # Proportional gain (tune for responsiveness)
KI = 0.1    # Integral gain (reduces steady-state error)
KD = 0.5    # Derivative gain (dampens oscillation)    

PID_MIN_OUTPUT = -200  # Maximum turn rate limit (left)
PID_MAX_OUTPUT = 200   # Maximum turn rate limit (right)  
INTEGRAL_MIN = -100.0  # Anti-windup: minimum integral value
INTEGRAL_MAX = 100.0   # Anti-windup: maximum integral value  

LOG_INTERVAL = 50 

# Wi-Fi streaming configuration for real-time plotting
ENABLE_STREAMING = True  # Set to False to disable Wi-Fi streaming
PC_IP = "192.168.1.100"  # CHANGE THIS to your PC's IP address
PC_PORT = 5005           # Port number for data streaming
STREAM_INTERVAL = 5      # Send data every N steps (to reduce network load)

ev3 = EV3Brick()

left_motor = Motor(LEFT_MOTOR_PORT)
right_motor = Motor(RIGHT_MOTOR_PORT)

robot = DriveBase(left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)

left_color_sensor = ColorSensor(LEFT_COLOR_PORT)
right_color_sensor = ColorSensor(RIGHT_COLOR_PORT)

pid_integral = 0.0
pid_previous_error = 0.0

step_count = 0
both_black_count = 0  # Counter for consecutive both-black detections

# Wi-Fi socket for streaming data
sock = None
if ENABLE_STREAMING:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print("Wi-Fi streaming enabled - sending to " + PC_IP + ":" + str(PC_PORT))
    except Exception as e:
        print("Warning: Could not create socket - " + str(e))
        sock = None

def read_sensors():
    
    left_r = left_color_sensor.reflection()
    right_r = right_color_sensor.reflection()
    return left_r, right_r


def pid_compute(setpoint, measured_value, dt):
    
    global pid_integral, pid_previous_error
    # error is the entry
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
    # outoutput is the iesire
    output = p_term + i_term + d_term

    if output > PID_MAX_OUTPUT:
        output = PID_MAX_OUTPUT
    elif output < PID_MIN_OUTPUT:
        output = PID_MIN_OUTPUT

    pid_previous_error = error
    
    return error, output  # Return both error and output for streaming

def pid_reset():
    global pid_integral, pid_previous_error
    pid_integral = 0.0
    pid_previous_error = 0.0

def send_data(step, error, output):
    """Send PID data over Wi-Fi to PC for plotting"""
    global sock
    if sock is not None:
        try:
            # Format: step,error,output
            data = str(step) + "," + str(error) + "," + str(output) + "\n"
            sock.sendto(data.encode(), (PC_IP, PC_PORT))
        except Exception as e:
            # Silently ignore network errors to keep robot running
            pass

def main():
    global step_count, both_black_count

    ev3.speaker.beep()
    print("\n" + "="*50)
    print("LANE KEEPING - PID LINE FOLLOWING")
    print("="*50)
    print("IMPORTANT: Position robot in the WHITE LANE between")
    print("           the two BLACK lines - both sensors on white")
    print("="*50)
    print("Configuration:")
    print("  Control mode: PID + Emergency Recovery")
    print("  Black threshold: " + str(BLACK_THRESHOLD) + " (for stop detection)")
    print("  Crossing threshold: " + str(CROSSING_THRESHOLD) + " (emergency line detection)")
    print("  Min difference: " + str(MIN_DIFFERENCE) + " (to trigger PID steering)")
    print("  PID gains: Kp=" + str(KP) + ", Ki=" + str(KI) + ", Kd=" + str(KD))
    print("  Speeds: Straight=" + str(BASE_SPEED) + " | Turn=" + str(TURNING_SPEED) + " | Emergency=" + str(EMERGENCY_SPEED) + " mm/s")
    print("  Turn rate limits: " + str(PID_MIN_OUTPUT) + " to " + str(PID_MAX_OUTPUT) + " | Emergency=" + str(EMERGENCY_TURN) + " deg/s")
    print("  Stop confirmation: " + str(STOP_CONFIRM_COUNT) + " consecutive readings")
    print("  Left color sensor: Port S2")
    print("  Right color sensor: Port S3")
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

            left_r, right_r = read_sensors()

            if step_count % LOG_INTERVAL == 0:
                print("Step " + str(step_count) + " | Left: " + str(left_r) + 
                      " | Right: " + str(right_r))

            left_on_black = left_r < BLACK_THRESHOLD
            right_on_black = right_r < BLACK_THRESHOLD
            
            # Stop condition: both sensors on black for multiple consecutive readings
            if left_on_black and right_on_black:
                both_black_count += 1
                if both_black_count >= STOP_CONFIRM_COUNT:
                    robot.stop()
                    ev3.speaker.beep()
                    print("\n=== BOTH SENSORS ON BLACK LINE (confirmed " + str(both_black_count) + "x) - STOPPING ===")
                    print("End of track detected!")
                    break
                else:
                    # Confirming stop - move slowly forward while counting
                    robot.drive(10, 0)
                    if step_count % LOG_INTERVAL == 0:
                        print("  -> Both on BLACK - Confirming " + str(both_black_count) + "/" + str(STOP_CONFIRM_COUNT))
                    wait(20)
                    continue  # Skip normal control logic while confirming stop
            else:
                both_black_count = 0  # Reset counter if not both black

            # Comparative control: steer based on which sensor is darker
            difference = left_r - right_r
            
            # Emergency recovery: if a sensor is on black line, turn away aggressively
            if left_r < CROSSING_THRESHOLD and right_r >= CROSSING_THRESHOLD:
                # Left sensor crossed line - turn RIGHT aggressively
                turn_rate = EMERGENCY_TURN
                speed = EMERGENCY_SPEED
                if step_count % LOG_INTERVAL == 0:
                    print("  -> EMERGENCY! Left on line (" + str(left_r) + ") - SHARP RIGHT")
            elif right_r < CROSSING_THRESHOLD and left_r >= CROSSING_THRESHOLD:
                # Right sensor crossed line - turn LEFT aggressively
                turn_rate = -EMERGENCY_TURN
                speed = EMERGENCY_SPEED
                if step_count % LOG_INTERVAL == 0:
                    print("  -> EMERGENCY! Right on line (" + str(right_r) + ") - SHARP LEFT")
            elif abs(difference) < MIN_DIFFERENCE:
                # Sensors reading similar values - go straight at full speed
                turn_rate = 0
                speed = BASE_SPEED
                if step_count % LOG_INTERVAL == 0:
                    print("  -> Balanced (diff=" + str(int(difference)) + ") - STRAIGHT at " + str(speed) + "mm/s")
            else:
                # PID control based on sensor difference
                # Target is 0 (balanced sensors), measured value is the difference
                # When left darker (negative diff), PID outputs positive → turn right
                error, turn_rate = pid_compute(0, difference, dt)
                
                # Send data for real-time plotting
                if ENABLE_STREAMING and step_count % STREAM_INTERVAL == 0:
                    send_data(step_count, error, turn_rate)
                
                # Reduce speed when turning to prevent overshoot
                speed = TURNING_SPEED
                
                if step_count % LOG_INTERVAL == 0:
                    direction = "RIGHT" if turn_rate > 0 else "LEFT"
                    print("  -> Diff=" + str(int(difference)) + " - PID Turning " + direction + 
                          " (" + str(int(turn_rate)) + "°/s) at " + str(speed) + "mm/s")

            # Ensure forward motion only (prevent backward turning)
            if speed < 10:
                speed = 10
            
            robot.drive(speed, turn_rate)

            wait(20)
    
    except KeyboardInterrupt:
        robot.stop()
        print("\n" + "="*50)
        print("LANE KEEPING STOPPED")
        print("Total steps: " + str(step_count))
        print("="*50 + "\n")
        ev3.speaker.beep()

if __name__ == "__main__":
    main()
