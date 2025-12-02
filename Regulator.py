from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

class Regulator:
    """
    Abstract base class for motor regulators.
    Defines interface for PID, Fuzzy, and other control algorithms.
    """
    
    def __init__(self):
        """Initialize the regulator"""
        pass
    
    def compute(self, setpoint, measured_value, dt):
        """
        Compute control output based on setpoint and measured value.
        
        Args:
            setpoint: Desired target value
            measured_value: Current measured value
            dt: Time delta since last computation (seconds)
            
        Returns:
            control_output: Control signal to apply to actuator
        """
        raise NotImplementedError("Subclasses must implement compute()")
    
    def reset(self):
        """
        Reset regulator state (integral terms, previous errors, etc.)
        Call this when starting a new control sequence.
        """
        raise NotImplementedError("Subclasses must implement reset()")
    
    def set_limits(self, min_output, max_output):
        """
        Set output limits for the control signal.
        
        Args:
            min_output: Minimum control output
            max_output: Maximum control output
        """
        raise NotImplementedError("Subclasses must implement set_limits()")
