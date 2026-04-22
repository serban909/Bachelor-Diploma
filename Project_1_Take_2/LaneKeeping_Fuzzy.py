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
# Mode: both sensors straddle the black line on a white surface.
# Normal state: both sensors read white (high reflection).
# Drift right -> right sensor approaches line -> right_r drops -> turn left.
# Drift left  -> left sensor approaches line  -> left_r drops  -> turn right.
BLACK_THRESHOLD = 6     # Absolute black line detection (for stop condition)
CROSSING_THRESHOLD = 9  # Emergency: sensor is on the black line
MIN_DIFFERENCE = 3      # Minimum sensor difference to trigger fuzzy steering
BASE_SPEED = 50         # Base forward speed in mm/s when going straight
TURNING_SPEED = 30      # Reduced speed when steering (to prevent overshoot)
EMERGENCY_SPEED = 20    # Very slow when recovering from line crossing
EMERGENCY_TURN = 35     # Recovery turn rate (kept low to avoid overshooting)
STOP_CONFIRM_COUNT = 3  # Number of consecutive readings needed to confirm stop

# Fuzzy PI base gains - adapted based on error magnitude
KP_BASE = 0.8   # Scaled so KP*typical_error stays within output limits
KI_BASE = 0.05  # Gentle integral to reduce steady-state offset

ERROR_SMALL = 3.0
ERROR_MEDIUM = 6.0
ERROR_LARGE = 10.0

FUZZY_MIN_OUTPUT = -40
FUZZY_MAX_OUTPUT = 40
INTEGRAL_MIN = -50.0
INTEGRAL_MAX = 50.0

LOG_INTERVAL = 50 

# Wi-Fi streaming configuration for real-time plotting
ENABLE_STREAMING = True  # Set to False to disable Wi-Fi streaming
PC_IP = "10.194.244.90"  # CHANGE THIS to your PC's IP address
PC_PORT = 5005           # Port number for data streaming

ev3 = EV3Brick()

left_motor = Motor(LEFT_MOTOR_PORT)
right_motor = Motor(RIGHT_MOTOR_PORT)

robot = DriveBase(left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)

left_color_sensor = ColorSensor(LEFT_COLOR_PORT)
right_color_sensor = ColorSensor(RIGHT_COLOR_PORT)

fuzzy_integral = 0.0

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

def fuzzify_error(error):
    
    error_abs = abs(error)
    
    if error_abs <= ERROR_SMALL:
        mu_small = 1.0
    elif error_abs <= ERROR_MEDIUM:
        mu_small = (ERROR_MEDIUM - error_abs) / (ERROR_MEDIUM - ERROR_SMALL)
    else:
        mu_small = 0.0

    if error_abs <= ERROR_SMALL:
        mu_medium = 0.0
    elif error_abs <= ERROR_MEDIUM:
        mu_medium = (error_abs - ERROR_SMALL) / (ERROR_MEDIUM - ERROR_SMALL)
    elif error_abs <= ERROR_LARGE:
        mu_medium = (ERROR_LARGE - error_abs) / (ERROR_LARGE - ERROR_MEDIUM)
    else:
        mu_medium = 0.0

    if error_abs <= ERROR_MEDIUM:
        mu_large = 0.0
    elif error_abs <= ERROR_LARGE:
        mu_large = (error_abs - ERROR_MEDIUM) / (ERROR_LARGE - ERROR_MEDIUM)
    else:
        mu_large = 1.0
    
    return (mu_small, mu_medium, mu_large)

def fuzzy_inference(mu_small, mu_medium, mu_large):

    kp_small = 0.6
    ki_small = 0.01
    
    kp_medium = 0.8
    ki_medium = 0.05
    
    kp_large = 1.0
    ki_large = 0.09
    
    total_weight = mu_small + mu_medium + mu_large
    
    if total_weight > 0:
        kp_multiplier = (mu_small * kp_small + mu_medium * kp_medium + mu_large * kp_large) / total_weight
        ki_multiplier = (mu_small * ki_small + mu_medium * ki_medium + mu_large * ki_large) / total_weight
    else:
        kp_multiplier = 1.0
        ki_multiplier = 1.0
    
    kp_adaptive = KP_BASE * kp_multiplier
    ki_adaptive = KI_BASE * ki_multiplier
    
    return (kp_adaptive, ki_adaptive)


def fuzzy_compute(error, dt):
    global fuzzy_integral
    
    mu_small, mu_medium, mu_large = fuzzify_error(error)
    
    kp_adaptive, ki_adaptive = fuzzy_inference(mu_small, mu_medium, mu_large)
    
    p_term = kp_adaptive * error
    
    if dt > 0:
        fuzzy_integral += error * dt
        if fuzzy_integral > INTEGRAL_MAX:
            fuzzy_integral = INTEGRAL_MAX
        elif fuzzy_integral < INTEGRAL_MIN:
            fuzzy_integral = INTEGRAL_MIN
    i_term = ki_adaptive * fuzzy_integral
    
    output = p_term + i_term
    
    if output > FUZZY_MAX_OUTPUT:
        output = FUZZY_MAX_OUTPUT
    elif output < FUZZY_MIN_OUTPUT:
        output = FUZZY_MIN_OUTPUT
    
    return error, output  # Return both error and output for streaming


def fuzzy_reset():
    global fuzzy_integral
    fuzzy_integral = 0.0

def send_data(step, error, output):
    """Send Fuzzy PI data over Wi-Fi to PC for plotting"""
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
    print("LANE KEEPING - PI-FUZZY LINE FOLLOWING")
    print("="*50)
    print("IMPORTANT: Position robot so the BLACK LINE is between")
    print("           the two sensors - both sensors on WHITE")
    print("="*50)
    print("Configuration:")
    print("  Mode: BLACK LINE Following - FUZZY PI + Emergency Recovery")
    print("  Black threshold: " + str(BLACK_THRESHOLD) + " (for stop detection)")
    print("  Crossing threshold: " + str(CROSSING_THRESHOLD) + " (emergency line detection)")
    print("  Min difference: " + str(MIN_DIFFERENCE) + " (to trigger fuzzy steering)")
    print("  PI-Fuzzy base gains: Kp=" + str(KP_BASE) + ", Ki=" + str(KI_BASE))
    print("  Fuzzy thresholds: Small=" + str(ERROR_SMALL) + ", Medium=" + str(ERROR_MEDIUM) + ", Large=" + str(ERROR_LARGE))
    print("  Speeds: Straight=" + str(BASE_SPEED) + " | Turn=" + str(TURNING_SPEED) + " | Emergency=" + str(EMERGENCY_SPEED) + " mm/s")
    print("  Turn rate limits: " + str(FUZZY_MIN_OUTPUT) + " to " + str(FUZZY_MAX_OUTPUT) + " | Emergency=" + str(EMERGENCY_TURN) + " deg/s")
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

            # Comparative control with fuzzy logic and emergency recovery
            difference = left_r - right_r
            
            # Emergency recovery: if a sensor is on black line, turn away aggressively
            if left_r < CROSSING_THRESHOLD and right_r >= CROSSING_THRESHOLD:
                # Left sensor on line -> robot drifted right -> turn left
                turn_rate = -EMERGENCY_TURN
                speed = EMERGENCY_SPEED
                if step_count % LOG_INTERVAL == 0:
                    print("  -> EMERGENCY! Left on line (" + str(left_r) + ") - SHARP LEFT")
            elif right_r < CROSSING_THRESHOLD and left_r >= CROSSING_THRESHOLD:
                # Right sensor on line -> robot drifted left -> turn right
                turn_rate = EMERGENCY_TURN
                speed = EMERGENCY_SPEED
                if step_count % LOG_INTERVAL == 0:
                    print("  -> EMERGENCY! Right on line (" + str(right_r) + ") - SHARP RIGHT")
            elif abs(difference) < MIN_DIFFERENCE:
                # Sensors reading similar values - go straight at full speed
                turn_rate = 0
                speed = BASE_SPEED
                if step_count % LOG_INTERVAL == 0:
                    print("  -> Balanced (diff=" + str(int(difference)) + ") - STRAIGHT at " + str(speed) + "mm/s")
            else:
                # Use fuzzy logic for adaptive steering based on difference.
                # error = -difference: when left darker (diff>0) → error<0 → turn left;
                # when right darker (diff<0) → error>0 → turn right.
                _, turn_rate = fuzzy_compute(-difference, dt)

                # Reduce speed when turning to prevent overshoot
                speed = TURNING_SPEED

                if step_count % LOG_INTERVAL == 0:
                    direction = "RIGHT" if turn_rate > 0 else "LEFT"
                    print("  -> Diff=" + str(int(difference)) + " - Fuzzy Turning " + direction +
                          " (" + str(int(turn_rate)) + "°/s) at " + str(speed) + "mm/s")

            # Stream every step so the plot has no gaps.
            # error = -difference (setpoint 0, measured is difference).
            if ENABLE_STREAMING:
                send_data(step_count, -difference, turn_rate)

            # Ensure forward motion only (prevent backward movement)
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
