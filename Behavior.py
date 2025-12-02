from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

class Behavior:
    def on_start(self, hardware_adapter):
        pass
    
    def step(self, hardware_adapter, dt):
        raise NotImplementedError
    
    def on_stop(self, hardware_adapter):
        pass