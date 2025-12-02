from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from Behavior import Behavior
from MazeSolver import MazeSolver
from LaneKeeping import LaneKeeping

class BehaviorFactory:
    @staticmethod
    def create(behavior_name, **kwargs):
        behavior_name = behavior_name.lower()
        if behavior_name == "maze":
            return MazeSolver(**kwargs)
        elif behavior_name == "lane":
            return LaneKeeping(**kwargs)
        else:
            raise ValueError("Unknown behavior: " + behavior_name)