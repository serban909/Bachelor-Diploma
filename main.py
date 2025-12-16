#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from HardwareAdapter import HardwareAdapter
from BehaviorFactory import BehaviorFactory


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

def main():
    hw=HardwareAdapter()
    ev3=hw.ev3
    
    ev3.screen.clear()
    ev3.screen.print("Pick:")
    ev3.screen.print("Left = Lane")
    ev3.screen.print("Right = Maze")
    ev3.screen.print("Center = Quit")
    
    while True:
        buttons=ev3.buttons.pressed()
        if buttons:
            button=buttons[0]
            if button==Button.LEFT:
                behavior_name="lane"
                break
            elif button==Button.RIGHT:
                behavior_name="maze"
                break
            elif button==Button.CENTER:
                ev3.screen.clear()
                ev3.screen.print("Exit")
                return
        wait(50)
        
    if behavior_name=="lane":
        # LaneKeeping with PID regulator (gentle tuning for smooth line following)
        behavior=BehaviorFactory.create("lane", 
                                       target_reflect=40, 
                                       base_speed=15, 
                                       kp=1.2, 
                                       threshold=28,
                                       regulator_type="PID",
                                       regulator_kp=2.0,
                                       regulator_ki=0.1,
                                       regulator_kd=0.2)
    elif behavior_name=="maze":
        # MazeSolver with optimized dual-sensor decision-making + sensor filtering
        behavior=BehaviorFactory.create("maze", 
                                       distance_threshold_low=30,   # Minimum acceptable distance from left wall
                                       distance_threshold_high=50,  # Maximum acceptable distance from left wall
                                       base_speed=25,               # Reduced speed for better control
                                       min_forward_distance=60,     # Approach distance - slow down at 60mm
                                       obstacle_threshold=30,       # Critical distance - turn at 30-35mm
                                       regulator_type="PID",
                                       regulator_kp=6.0,            # Moderate for smooth wall tracking (was 10.0, reduced for stability)
                                       regulator_ki=0.02,           # Eliminate steady-state error (increased slightly)
                                       regulator_kd=0.05)           # Dampen oscillations (increased for stability)
        
    behavior.on_start(hw)
    
    stopwatch=StopWatch()
    stopwatch.reset()
    last=stopwatch.time()/1000.0
    
    try:
        while True:
            now= stopwatch.time()/1000.0
            dt=now-last
            if dt<=0:
                dt=0.1
            last=now
            behavior.step(hw, dt)
            wait(20)
    except KeyboardInterrupt:
        behavior.on_stop(hw)
        hw.stop()
        
if __name__=="__main__":
    main()


#ev3 = EV3Brick()
#test_motor=Motor(Port.B)

#left_motor = Motor(Port.A)
#right_motor = Motor(Port.C)

#ultrasonic_sensor = UltrasonicSensor(Port.S4)

#robot = DriveBase(Motor(Port.A), Motor(Port.C), wheel_diameter=56, axle_track=122)

#while True:
 #   distance = ultrasonic_sensor.distance()
  #  print("Distance:", distance, " mm")
   # if distance < 300:
    #    robot.stop()
     #   robot.turn(90)
    #else:
    #    robot.drive(9, 0)
    #wait(10)

#drive_base.stop()

#ev3.speaker.beep()

#test_motor.run_target(500, 90)

#left_motor.run_target(500, 500)
#right_motor.run_target(500, 500)


# Write your program here.
#ev3.speaker.beep(1000, 500)
