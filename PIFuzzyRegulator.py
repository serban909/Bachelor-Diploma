from Regulator import Regulator

class PIFuzzyRegulator(Regulator):
    """
    PI-Fuzzy Controller (Proportional-Integral with Fuzzy Logic).
    
    Combines:
    - Proportional control for immediate response
    - Integral control for steady-state error elimination
    - Fuzzy logic for adaptive gain adjustment
    
    Fuzzy membership functions adjust Kp and Ki based on error magnitude.
    """
    
    def __init__(self, kp_base, ki_base, min_output=-100, max_output=100):
        """
        Initialize PI-Fuzzy controller.
        
        Args:
            kp_base: Base proportional gain (required, will be adjusted by fuzzy logic)
            ki_base: Base integral gain (required, will be adjusted by fuzzy logic)
            min_output: Minimum control output
            max_output: Maximum control output
        """
        super().__init__()
        self.kp_base = kp_base
        self.ki_base = ki_base
        
        # State variables
        self.integral = 0.0
        self.previous_error = 0.0
        
        # Output limits
        self.min_output = min_output
        self.max_output = max_output
        
        # Anti-windup
        self.integral_max = 1000.0
        self.integral_min = -1000.0
        
        # Fuzzy logic parameters
        # Error ranges for membership functions
        self.error_small = 5.0    # Small error threshold
        self.error_medium = 15.0  # Medium error threshold
        self.error_large = 30.0   # Large error threshold
    
    def _fuzzify_error(self, error):
        """
        Fuzzify the error into membership degrees.
        
        Returns membership degrees for: small, medium, large error
        
        Args:
            error: Absolute error value
            
        Returns:
            Tuple of (mu_small, mu_medium, mu_large)
        """
        error_abs = abs(error)
        
        # Small error membership (triangular)
        if error_abs <= self.error_small:
            mu_small = 1.0
        elif error_abs <= self.error_medium:
            mu_small = (self.error_medium - error_abs) / (self.error_medium - self.error_small)
        else:
            mu_small = 0.0
        
        # Medium error membership (triangular)
        if error_abs <= self.error_small:
            mu_medium = 0.0
        elif error_abs <= self.error_medium:
            mu_medium = (error_abs - self.error_small) / (self.error_medium - self.error_small)
        elif error_abs <= self.error_large:
            mu_medium = (self.error_large - error_abs) / (self.error_large - self.error_medium)
        else:
            mu_medium = 0.0
        
        # Large error membership (trapezoidal)
        if error_abs <= self.error_medium:
            mu_large = 0.0
        elif error_abs <= self.error_large:
            mu_large = (error_abs - self.error_medium) / (self.error_large - self.error_medium)
        else:
            mu_large = 1.0
        
        return (mu_small, mu_medium, mu_large)
    
    def _fuzzy_inference(self, mu_small, mu_medium, mu_large):
        """
        Apply fuzzy rules to determine gain adjustments.
        
        Fuzzy Rules:
        1. IF error is SMALL THEN Kp is SMALL and Ki is MEDIUM
        2. IF error is MEDIUM THEN Kp is MEDIUM and Ki is SMALL
        3. IF error is LARGE THEN Kp is LARGE and Ki is VERY_SMALL
        
        Args:
            mu_small, mu_medium, mu_large: Membership degrees
            
        Returns:
            Tuple of (kp_factor, ki_factor) multipliers for base gains
        """
        # Define fuzzy outputs for Kp (proportional gain factor)
        kp_small = 0.5
        kp_medium = 1.0
        kp_large = 1.5
        
        # Define fuzzy outputs for Ki (integral gain factor)
        ki_very_small = 0.1
        ki_small = 0.5
        ki_medium = 1.0
        
        # Apply fuzzy rules using weighted average defuzzification
        kp_numerator = (mu_small * kp_small + 
                       mu_medium * kp_medium + 
                       mu_large * kp_large)
        
        ki_numerator = (mu_small * ki_medium + 
                       mu_medium * ki_small + 
                       mu_large * ki_very_small)
        
        denominator = mu_small + mu_medium + mu_large
        
        # Avoid division by zero
        if denominator > 0:
            kp_factor = kp_numerator / denominator
            ki_factor = ki_numerator / denominator
        else:
            kp_factor = 1.0
            ki_factor = 1.0
        
        return (kp_factor, ki_factor)
    
    def compute(self, setpoint, measured_value, dt):
        """
        Compute PI-Fuzzy control output.
        
        Args:
            setpoint: Desired target value
            measured_value: Current measured value
            dt: Time delta since last computation (seconds)
            
        Returns:
            control_output: PI-Fuzzy control signal
        """
        # Calculate error
        error = setpoint - measured_value
        
        # Fuzzify error
        mu_small, mu_medium, mu_large = self._fuzzify_error(error)
        
        # Apply fuzzy inference to adjust gains
        kp_factor, ki_factor = self._fuzzy_inference(mu_small, mu_medium, mu_large)
        
        # Calculate adaptive gains
        kp = self.kp_base * kp_factor
        ki = self.ki_base * ki_factor
        
        # Proportional term
        p_term = kp * error
        
        # Integral term (with anti-windup)
        if dt > 0:
            self.integral += error * dt
            # Clamp integral
            if self.integral > self.integral_max:
                self.integral = self.integral_max
            elif self.integral < self.integral_min:
                self.integral = self.integral_min
        i_term = ki * self.integral
        
        # Calculate total output
        output = p_term + i_term
        
        # Apply output limits
        if output > self.max_output:
            output = self.max_output
        elif output < self.min_output:
            output = self.min_output
        
        # Save error for next iteration
        self.previous_error = error
        
        return output
    
    def reset(self):
        """Reset PI-Fuzzy controller state."""
        self.integral = 0.0
        self.previous_error = 0.0
    
    def set_limits(self, min_output, max_output):
        """Set output limits."""
        self.min_output = min_output
        self.max_output = max_output
    
    def set_integral_limits(self, min_integral, max_integral):
        """Set integral term limits for anti-windup."""
        self.integral_min = min_integral
        self.integral_max = max_integral
    
    def set_base_gains(self, kp_base, ki_base):
        """
        Update base PI gains.
        
        Args:
            kp_base: New base proportional gain
            ki_base: New base integral gain
        """
        self.kp_base = kp_base
        self.ki_base = ki_base
    
    def set_fuzzy_thresholds(self, small, medium, large):
        """
        Adjust fuzzy membership function thresholds.
        
        Args:
            small: Small error threshold
            medium: Medium error threshold
            large: Large error threshold
        """
        self.error_small = small
        self.error_medium = medium
        self.error_large = large
    
    def get_state(self):
        """
        Get current controller state for debugging.
        
        Returns:
            Dictionary with current state values
        """
        return {
            'kp_base': self.kp_base,
            'ki_base': self.ki_base,
            'integral': self.integral,
            'previous_error': self.previous_error,
            'output_limits': (self.min_output, self.max_output),
            'fuzzy_thresholds': (self.error_small, self.error_medium, self.error_large)
        }
