from Regulator import Regulator

class PIDRegulator(Regulator):
    """
    PID (Proportional-Integral-Derivative) Controller.
    
    Standard PID control algorithm:
    output = Kp * error + Ki * integral(error) + Kd * derivative(error)
    
    Where:
    - Kp: Proportional gain (response to current error)
    - Ki: Integral gain (response to accumulated error)
    - Kd: Derivative gain (response to error rate of change)
    """
    
    def __init__(self, kp, ki, kd, min_output=-100, max_output=100):
        """
        Initialize PID controller.
        
        Args:
            kp: Proportional gain (required)
            ki: Integral gain (required)
            kd: Derivative gain (required)
            min_output: Minimum control output (default: -100)
            max_output: Maximum control output (default: 100)
        """
        super().__init__()
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # State variables
        self.integral = 0.0
        self.previous_error = 0.0
        
        # Output limits
        self.min_output = min_output
        self.max_output = max_output
        
        # Anti-windup: limit integral term
        self.integral_max = 1000.0
        self.integral_min = -1000.0
    
    def compute(self, setpoint, measured_value, dt):
        """
        Compute PID control output.
        
        Args:
            setpoint: Desired target value
            measured_value: Current measured value
            dt: Time delta since last computation (seconds)
            
        Returns:
            control_output: PID control signal
        """
        # Calculate error
        error = setpoint - measured_value
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term (with anti-windup)
        if dt > 0:
            self.integral += error * dt
            # Clamp integral to prevent windup
            if self.integral > self.integral_max:
                self.integral = self.integral_max
            elif self.integral < self.integral_min:
                self.integral = self.integral_min
        i_term = self.ki * self.integral
        
        # Derivative term
        if dt > 0:
            derivative = (error - self.previous_error) / dt
        else:
            derivative = 0.0
        d_term = self.kd * derivative
        
        # Calculate total output
        output = p_term + i_term + d_term
        
        # Apply output limits
        if output > self.max_output:
            output = self.max_output
        elif output < self.min_output:
            output = self.min_output
        
        # Save error for next iteration
        self.previous_error = error
        
        return output
    
    def reset(self):
        """Reset PID controller state."""
        self.integral = 0.0
        self.previous_error = 0.0
    
    def set_limits(self, min_output, max_output):
        """Set output limits."""
        self.min_output = min_output
        self.max_output = max_output
    
    def set_integral_limits(self, min_integral, max_integral):
        """
        Set integral term limits for anti-windup.
        
        Args:
            min_integral: Minimum integral value
            max_integral: Maximum integral value
        """
        self.integral_min = min_integral
        self.integral_max = max_integral
    
    def set_gains(self, kp, ki, kd):
        """
        Update PID gains during runtime.
        
        Args:
            kp: New proportional gain
            ki: New integral gain
            kd: New derivative gain
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
    
    def get_state(self):
        """
        Get current controller state for debugging.
        
        Returns:
            Dictionary with current state values
        """
        return {
            'kp': self.kp,
            'ki': self.ki,
            'kd': self.kd,
            'integral': self.integral,
            'previous_error': self.previous_error,
            'output_limits': (self.min_output, self.max_output)
        }
