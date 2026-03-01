#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait
from pybricks.robotics import DriveBase

LEFT_MOTOR_PORT = Port.A
RIGHT_MOTOR_PORT = Port.C
FORWARD_ULTRASONIC_PORT = Port.S4 
LEFT_ULTRASONIC_PORT = Port.S1  

WHEEL_DIAMETER = 55.5 
AXLE_TRACK = 104      

TURN_INCREMENTS = [15, 30, 45, 60, 90]  
READINGS_PER_TEST = 5  

ev3 = EV3Brick()

left_motor = Motor(LEFT_MOTOR_PORT)
right_motor = Motor(RIGHT_MOTOR_PORT)

robot = DriveBase(left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)

forward_sensor = UltrasonicSensor(FORWARD_ULTRASONIC_PORT)
left_sensor = UltrasonicSensor(LEFT_ULTRASONIC_PORT)

def read_and_print_sensors(step_num):
    forward_dist = forward_sensor.distance()
    left_dist = left_sensor.distance()
    
    print("Step " + str(step_num) + " | Forward: " + str(forward_dist) + "mm | Left: " + str(left_dist) + "mm")
    
    return forward_dist, left_dist

def test_continuous_reading():
    print("\n" + "="*60)
    print("CONTINUOUS SENSOR READING TEST")
    print("="*60)
    print("Press CENTER button to stop and proceed to turn tests")
    print("="*60 + "\n")
    
    step = 0
    while True:
        step += 1
        read_and_print_sensors(step)
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
    print("  - Smaller angles (15-30°): More precise but slower")
    print("  - Medium angles (45-60°): Balanced precision/speed")
    print("  - Large angles (90°): Fast but less precise")
    print("\nChoose the smallest angle that gives consistent readings.")
    print("="*60 + "\n")

def main():

    ev3.speaker.beep()
    print("\n" + "="*60)
    print("ULTRASONIC SENSOR TEST")
    print("="*60)
    print("Hardware Configuration:")
    print("  Forward sensor: Port S4")
    print("  Left sensor: Port S1")
    print("  Left motor: Port A")
    print("  Right motor: Port C")
    print("\nTest Modes:")
    print("  1. Continuous reading (500ms interval)")
    print("  2. Turn increment tests: " + str(TURN_INCREMENTS))
    print("="*60 + "\n")
    
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
