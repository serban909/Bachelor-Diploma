#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait
from pybricks.robotics import DriveBase

LEFT_MOTOR_PORT = Port.A
RIGHT_MOTOR_PORT = Port.C
LEFT_COLOR_PORT = Port.S2
RIGHT_COLOR_PORT = Port.S3

WHEEL_DIAMETER = 55.5 
AXLE_TRACK = 104       

TURN_INCREMENTS = [15, 30, 45, 60, 90]  
READINGS_PER_TEST = 5  

ev3 = EV3Brick()

left_motor = Motor(LEFT_MOTOR_PORT)
right_motor = Motor(RIGHT_MOTOR_PORT)

robot = DriveBase(left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)

left_color_sensor = ColorSensor(LEFT_COLOR_PORT)
right_color_sensor = ColorSensor(RIGHT_COLOR_PORT)

def read_and_print_sensors(step_num):
    left_reflection = left_color_sensor.reflection()
    right_reflection = right_color_sensor.reflection()
    
    print("Step " + str(step_num) + " | Left: " + str(left_reflection) + " | Right: " + str(right_reflection))
    
    return left_reflection, right_reflection

def test_continuous_reading():
    print("\n" + "="*60)
    print("CONTINUOUS SENSOR READING TEST")
    print("="*60)
    print("Press CENTER button to stop and proceed to turn tests")
    print("="*60 + "\n")
    
    step = 0
    while True:
        step += 1
        left_r, right_r = read_and_print_sensors(step)
 
        if step % 10 == 0:
            print("  -> Difference (R-L): " + str(right_r - left_r))
        
        wait(500) 
        
        if Button.CENTER in ev3.buttons.pressed():
            print("\nContinuous reading stopped.\n")
            ev3.speaker.beep()
            wait(1000)
            break

def test_turn_increment(angle):
    print("\n" + "="*60)
    print("TESTING TURN INCREMENT: " + str(angle) + " DEGREES")
    print("="*60)
    print("Taking " + str(READINGS_PER_TEST) + " readings at each position")
    print()
    
    total_angle = 0
    turn_count = 0
    
    print("Position 0 (Start):")
    for i in range(READINGS_PER_TEST):
        read_and_print_sensors(i + 1)
        wait(200)
    
    for turn_num in range(3):
        turn_count += 1
        total_angle += angle
        
        print("\n>> Turning " + str(angle) + " degrees left...")
        robot.turn(angle)
        wait(500) 
        
        print("Position " + str(turn_count) + " (Total: " + str(total_angle) + " degrees):")
        for i in range(READINGS_PER_TEST):
            read_and_print_sensors(i + 1)
            wait(200)
    
    print("\n>> Returning to start position...")
    robot.turn(-total_angle)
    wait(1000)
    
    print("\nTest complete for " + str(angle) + " degree increment!")
    print("Total turns made: " + str(turn_count))
    print("="*60)

def test_all_increments():
    print("\n" + "="*60)
    print("TURN INCREMENT COMPARISON TEST")
    print("="*60)
    print("Will test the following turn increments:")
    for angle in TURN_INCREMENTS:
        print("  - " + str(angle) + " degrees")
    print("\nPress CENTER button after each test to continue")
    print("="*60 + "\n")
    
    wait(2000)
    
    for angle in TURN_INCREMENTS:
        test_turn_increment(angle)
        
        print("\nPress CENTER button to test next increment...")
        ev3.speaker.beep()
        
        while True:
            if Button.CENTER in ev3.buttons.pressed():
                wait(500) 
                break
            wait(100)
    
    print("\n" + "="*60)
    print("ALL TURN INCREMENT TESTS COMPLETED!")
    print("="*60)
    print("\nRecommendations:")
    print("  - Smaller angles (15-30°): More precise adjustments")
    print("  - Medium angles (45-60°): Balanced correction speed")
    print("  - Large angles (90°): Quick course changes")
    print("\nFor line following, choose the smallest angle that")
    print("allows the robot to correct course without overshooting.")
    print("="*60 + "\n")

def calibration_helper():
    print("\n" + "="*60)
    print("CALIBRATION HELPER")
    print("="*60)
    print("Place robot on WHITE surface")
    print("Press CENTER button to record white values...")
    print("="*60 + "\n")
    
    while True:
        if Button.CENTER in ev3.buttons.pressed():
            wait(500)
            break
        wait(100)
    
    white_left = left_color_sensor.reflection()
    white_right = right_color_sensor.reflection()
    print("WHITE values recorded:")
    print("  Left: " + str(white_left))
    print("  Right: " + str(white_right))
    print()
    ev3.speaker.beep()
    wait(1000)
    
    print("Now place robot on BLACK line")
    print("Press CENTER button to record black values...")
    print()
    
    while True:
        if Button.CENTER in ev3.buttons.pressed():
            wait(500)
            break
        wait(100)
    
    black_left = left_color_sensor.reflection()
    black_right = right_color_sensor.reflection()
    print("BLACK values recorded:")
    print("  Left: " + str(black_left))
    print("  Right: " + str(black_right))
    print()
    ev3.speaker.beep()
    
    avg_white = (white_left + white_right) / 2.0
    avg_black = (black_left + black_right) / 2.0
    target = (avg_white + avg_black) / 2.0
    threshold = avg_black + (avg_white - avg_black) * 0.3
    
    print("\n" + "="*60)
    print("CALIBRATION RESULTS")
    print("="*60)
    print("Average WHITE: " + str(int(avg_white)))
    print("Average BLACK: " + str(int(avg_black)))
    print("\nRecommended settings:")
    print("  TARGET_REFLECTION = " + str(int(target)) + "  (edge of line)")
    print("  LINE_THRESHOLD = " + str(int(threshold)) + "  (on black line)")
    print("="*60 + "\n")

def main():

    ev3.speaker.beep()
    print("\n" + "="*60)
    print("COLOR SENSOR TEST")
    print("="*60)
    print("Hardware Configuration:")
    print("  Left color sensor: Port S2")
    print("  Right color sensor: Port S3")
    print("  Left motor: Port A")
    print("  Right motor: Port C")
    print("\nTest Modes:")
    print("  1. Calibration helper (white/black values)")
    print("  2. Continuous reading (500ms interval)")
    print("  3. Turn increment tests: " + str(TURN_INCREMENTS))
    print("="*60 + "\n")
    
    wait(2000)
    
    calibration_helper()
    wait(2000)
    
    test_continuous_reading()
    
    test_all_increments()
    
    ev3.speaker.beep(frequency=1000, duration=200)
    wait(200)
    ev3.speaker.beep(frequency=1200, duration=200)
    print("\nAll tests completed!")
    print("Robot is stationary. Safe to pick up.\n")

if __name__ == "__main__":
    main()
