# Regulator System Documentation

## Overview

The regulator system provides control algorithms for precise motor control. It follows the **Strategy Pattern** and **SOLID principles** for flexibility and maintainability.

## Architecture

### Class Hierarchy

```
Regulator (Abstract Base Class)
├── PIDRegulator (Classical PID Control)
└── PIFuzzyRegulator (Fuzzy Logic + PI Control)
```

## Classes

### 1. Regulator (Interface)

Abstract base class defining the regulator interface.

**Methods:**
- `compute(setpoint, measured_value, dt)` - Calculate control output
- `reset()` - Reset controller state
- `set_limits(min_output, max_output)` - Set output bounds

---

### 2. PIDRegulator

Classical PID (Proportional-Integral-Derivative) controller.

**Control Equation:**
```
output = Kp * error + Ki * ∫error dt + Kd * d(error)/dt
```

**Parameters:**
- `kp` - Proportional gain (immediate response to error)
- `ki` - Integral gain (eliminates steady-state error)
- `kd` - Derivative gain (dampens oscillations)

**Features:**
- Anti-windup protection (integral clamping)
- Configurable output limits
- Runtime gain adjustment
- State inspection for debugging

**Example Usage:**
```python
from PIDRegulator import PIDRegulator

# Create PID controller for speed control
speed_pid = PIDRegulator(kp=2.0, ki=0.5, kd=0.1, min_output=-100, max_output=100)

# In control loop
target_speed = 100  # mm/s
current_speed = 85  # mm/s
dt = 0.02  # 20ms

control_signal = speed_pid.compute(target_speed, current_speed, dt)
# Apply control_signal to motor
```

**Tuning Guidelines:**
1. Start with `kp` only (ki=0, kd=0)
2. Increase `kp` until system oscillates
3. Add `ki` to eliminate steady-state error
4. Add `kd` to reduce overshoot

---

### 3. PIFuzzyRegulator

Adaptive PI controller using fuzzy logic for gain scheduling.

**How It Works:**

1. **Fuzzification**: Converts error into fuzzy membership degrees
   - Small error (0-5 units)
   - Medium error (5-15 units)
   - Large error (15-30+ units)

2. **Fuzzy Rules**:
   - IF error is SMALL → Use small Kp, medium Ki (fine control)
   - IF error is MEDIUM → Use medium Kp, small Ki (balanced)
   - IF error is LARGE → Use large Kp, very small Ki (fast response)

3. **Defuzzification**: Calculates adaptive gains using weighted average

**Advantages over PID:**
- Automatic gain adaptation based on error magnitude
- Better performance across wide operating ranges
- More robust to system variations
- No derivative term (less noise sensitivity)

**Example Usage:**
```python
from PIFuzzyRegulator import PIFuzzyRegulator

# Create PI-Fuzzy controller for turning
turn_fuzzy = PIFuzzyRegulator(kp_base=1.5, ki_base=0.3, min_output=-200, max_output=200)

# Adjust fuzzy thresholds for your application
turn_fuzzy.set_fuzzy_thresholds(small=3.0, medium=10.0, large=25.0)

# In control loop
target_angle = 90  # degrees
current_angle = 45  # degrees
dt = 0.02

control_signal = turn_fuzzy.compute(target_angle, current_angle, dt)
# Apply control_signal to motors
```

**Tuning Guidelines:**
1. Set base gains (kp_base, ki_base) for medium errors
2. Adjust fuzzy thresholds based on typical error ranges
3. Test across different operating conditions
4. Fine-tune membership functions if needed

---

## Integration with Robot System

### Option 1: Direct Motor Control

```python
from PIDRegulator import PIDRegulator
from HardwareAdapter import HardwareAdapter

hw = HardwareAdapter()
speed_regulator = PIDRegulator(kp=2.0, ki=0.5, kd=0.1)

target_speed = 150  # mm/s
dt = 0.02

while True:
    current_speed = hw.robot.distance() / dt  # Calculate from encoder
    control_output = speed_regulator.compute(target_speed, current_speed, dt)
    hw.drive_forward(control_output)
    wait(20)
```

### Option 2: Enhanced HardwareAdapter

Create regulated versions of movement methods:

```python
class RegulatedHardwareAdapter(HardwareAdapter):
    def __init__(self, regulator_type="PID"):
        super().__init__()
        if regulator_type == "PID":
            self.regulator = PIDRegulator(kp=1.5, ki=0.3, kd=0.05)
        elif regulator_type == "PIFuzzy":
            self.regulator = PIFuzzyRegulator(kp_base=1.5, ki_base=0.3)
    
    def regulated_turn(self, target_angle, dt):
        current_angle = self.robot.angle()
        turn_rate = self.regulator.compute(target_angle, current_angle, dt)
        self.robot.drive(0, turn_rate)
```

### Option 3: Behavior-Level Integration

Add regulator to specific behaviors:

```python
class RegulatedMazeSolver(MazeSolver):
    def __init__(self, regulator_type="PIFuzzy", **kwargs):
        super().__init__(**kwargs)
        if regulator_type == "PID":
            self.turn_regulator = PIDRegulator(kp=2.0, ki=0.4, kd=0.1)
        else:
            self.turn_regulator = PIFuzzyRegulator(kp_base=2.0, ki_base=0.4)
```

---

## Comparison: PID vs PI-Fuzzy

| Feature | PID | PI-Fuzzy |
|---------|-----|----------|
| **Simplicity** | Simple, well-understood | More complex logic |
| **Tuning** | 3 parameters (Kp, Ki, Kd) | 2 base + fuzzy thresholds |
| **Adaptivity** | Fixed gains | Adaptive gains |
| **Noise Sensitivity** | Derivative term sensitive | No derivative, less noise |
| **Large Errors** | May overshoot | Aggressive response |
| **Small Errors** | Good precision | Excellent precision |
| **Computational Cost** | Low | Medium |
| **Best For** | Known, stable systems | Variable conditions |

---

## Performance Tuning

### PID Tuning Tips

**Proportional (Kp):**
- Too low: Slow response, steady-state error
- Too high: Overshoot, oscillations
- Start: 0.5-2.0

**Integral (Ki):**
- Too low: Persistent steady-state error
- Too high: Integral windup, slow response
- Start: 0.1-0.5

**Derivative (Kd):**
- Too low: Overshoot
- Too high: Noise amplification
- Start: 0.01-0.1

### PI-Fuzzy Tuning Tips

**Base Gains:**
- Similar to PID P and I terms
- Tune for medium error range

**Fuzzy Thresholds:**
- `small`: ~5-10% of typical range
- `medium`: ~15-25% of typical range
- `large`: ~30-50% of typical range

---

## Debugging

Both regulators provide `get_state()` method:

```python
state = regulator.get_state()
print("Current state:", state)
# Output: {'kp': 2.0, 'ki': 0.5, 'integral': 15.3, ...}
```

Log regulator state to track performance:

```python
logger.log_sensor_reading("regulator_output", control_signal)
logger.log_sensor_reading("regulator_error", error)
```

---

## Next Steps

1. **Test regulators** with simple motor control
2. **Choose integration approach** (HardwareAdapter or Behavior)
3. **Collect data** for ML training with regulator parameters
4. **Compare performance** between PID and PI-Fuzzy
5. **Optimize** based on real-world testing

---

## Future Enhancements

- **Cascaded Control**: Speed + position control
- **Adaptive PID**: Self-tuning using ML
- **Model Predictive Control (MPC)**: Advanced control
- **Kalman Filtering**: Sensor fusion for better feedback
