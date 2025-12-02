# Regulator Usage Guide

## Overview

The regulator system now requires behavior-specific PID parameters to be passed during instantiation. This allows fine-tuning control for different robot behaviors.

## Parameter Requirements

### PIDRegulator
**Required parameters:**
- `kp` - Proportional gain (required)
- `ki` - Integral gain (required)
- `kd` - Derivative gain (required)
- `min_output` - Minimum output limit (default: -100)
- `max_output` - Maximum output limit (default: 100)

### PIFuzzyRegulator
**Required parameters:**
- `kp_base` - Base proportional gain (required)
- `ki_base` - Base integral gain (required)
- `min_output` - Minimum output limit (default: -100)
- `max_output` - Maximum output limit (default: 100)

## Behavior Configuration

### LaneKeeping Parameters

**Default tuning for smooth line following:**
```python
regulator_type = "PID"
regulator_kp = 1.2    # Gentle proportional response
regulator_ki = 0.3    # Moderate integral action
regulator_kd = 0.05   # Small derivative damping
```

**Example instantiation:**
```python
lane_behavior = LaneKeeping(
    target_reflect=40,
    base_speed=140,
    kp=1.2,              # Line-following proportional gain
    threshold=28,
    regulator_type="PID",
    regulator_kp=1.2,    # Regulator proportional gain
    regulator_ki=0.3,    # Regulator integral gain
    regulator_kd=0.05    # Regulator derivative gain
)
```

**Using PI-Fuzzy instead:**
```python
lane_behavior = LaneKeeping(
    target_reflect=40,
    base_speed=140,
    kp=1.2,
    threshold=28,
    regulator_type="PIFuzzy",
    regulator_kp=1.2,     # Base Kp (will adapt)
    regulator_ki=0.3,     # Base Ki (will adapt)
    regulator_kd=0.0      # Not used in PI-Fuzzy
)
```

---

### MazeSolver Parameters

**Default tuning for quick turns:**
```python
regulator_type = "PID"
regulator_kp = 2.0    # Aggressive proportional response
regulator_ki = 0.4    # Moderate integral action
regulator_kd = 0.1    # Stronger derivative damping
```

**Example instantiation:**
```python
maze_behavior = MazeSolver(
    wall_distance_mm=60,
    forward_distance_mm=150,
    probe_turn_deg=90,
    probe_pause_ms=300,
    regulator_type="PID",
    regulator_kp=2.0,     # Regulator proportional gain
    regulator_ki=0.4,     # Regulator integral gain
    regulator_kd=0.1      # Regulator derivative gain
)
```

**Using PI-Fuzzy instead:**
```python
maze_behavior = MazeSolver(
    wall_distance_mm=60,
    forward_distance_mm=150,
    probe_turn_deg=90,
    probe_pause_ms=300,
    regulator_type="PIFuzzy",
    regulator_kp=2.0,     # Base Kp (will adapt to large errors)
    regulator_ki=0.4,     # Base Ki (will adapt to error size)
    regulator_kd=0.0      # Not used in PI-Fuzzy
)
```

---

## Using BehaviorFactory

The `BehaviorFactory.create()` method accepts all parameters including regulator settings:

### LaneKeeping via Factory
```python
from BehaviorFactory import BehaviorFactory

behavior = BehaviorFactory.create(
    "lane",
    target_reflect=40,
    base_speed=140,
    kp=1.2,
    threshold=28,
    regulator_type="PID",
    regulator_kp=1.2,
    regulator_ki=0.3,
    regulator_kd=0.05
)
```

### MazeSolver via Factory
```python
from BehaviorFactory import BehaviorFactory

behavior = BehaviorFactory.create(
    "maze",
    wall_distance_mm=60,
    forward_distance_mm=150,
    probe_turn_deg=90,
    probe_pause_ms=300,
    regulator_type="PIFuzzy",  # Using PI-Fuzzy instead
    regulator_kp=2.0,
    regulator_ki=0.4,
    regulator_kd=0.0
)
```

---

## Tuning Guidelines by Behavior

### LaneKeeping (Line Following)

**Characteristics:**
- Needs smooth, continuous control
- Small errors are common (edge detection)
- Should avoid oscillations

**Recommended ranges:**
- `Kp`: 0.8 - 1.5 (gentle response)
- `Ki`: 0.2 - 0.5 (slow integral buildup)
- `Kd`: 0.01 - 0.1 (light damping)

**Signs of poor tuning:**
- **Oscillating/zigzagging**: Reduce Kp or increase Kd
- **Missing the line**: Increase Kp
- **Overshooting curves**: Increase Kd
- **Steady-state offset**: Increase Ki

---

### MazeSolver (Wall Following/Navigation)

**Characteristics:**
- Needs quick, decisive turns
- Large errors during decision-making
- Precision less critical than speed

**Recommended ranges:**
- `Kp`: 1.5 - 2.5 (aggressive response)
- `Ki`: 0.3 - 0.6 (moderate integral)
- `Kd`: 0.05 - 0.15 (medium damping)

**Signs of poor tuning:**
- **Slow turns**: Increase Kp
- **Overshoot during turns**: Increase Kd
- **Not completing turns**: Increase Ki or Kp
- **Jerky movement**: Reduce Kp or increase Kd

---

## Switching Regulator Types

You can easily switch between PID and PI-Fuzzy:

### Option 1: Change regulator_type parameter
```python
# Change from PID to PI-Fuzzy
behavior = LaneKeeping(
    regulator_type="PIFuzzy",  # Just change this
    regulator_kp=1.2,
    regulator_ki=0.3,
    regulator_kd=0.0           # Will be ignored
)
```

### Option 2: Disable regulators (use simple control)
```python
# Use no regulator (direct motor control)
behavior = LaneKeeping(
    regulator_type="None",  # Or any invalid value
    regulator_kp=0.0,       # Will be ignored
    regulator_ki=0.0,
    regulator_kd=0.0
)
```

---

## Main.py Example

Complete example showing both behaviors with different regulator types:

```python
from HardwareAdapter import HardwareAdapter
from BehaviorFactory import BehaviorFactory
from pybricks.tools import wait, StopWatch

def main():
    hw = HardwareAdapter()
    
    # User selects behavior (left = lane, right = maze)
    # ... button selection code ...
    
    if behavior_name == "lane":
        # LaneKeeping with PID regulator
        behavior = BehaviorFactory.create("lane", 
            target_reflect=40, 
            base_speed=140, 
            kp=1.2, 
            threshold=28,
            regulator_type="PID",
            regulator_kp=1.2,
            regulator_ki=0.3,
            regulator_kd=0.05
        )
    elif behavior_name == "maze":
        # MazeSolver with PI-Fuzzy regulator
        behavior = BehaviorFactory.create("maze", 
            wall_distance_mm=60, 
            forward_distance_mm=150, 
            probe_turn_deg=90,
            regulator_type="PIFuzzy",
            regulator_kp=2.0,
            regulator_ki=0.4,
            regulator_kd=0.0
        )
    
    behavior.on_start(hw)
    
    stopwatch = StopWatch()
    stopwatch.reset()
    last = stopwatch.time() / 1000.0
    
    try:
        while True:
            now = stopwatch.time() / 1000.0
            dt = now - last
            if dt <= 0:
                dt = 0.01
            last = now
            behavior.step(hw, dt)
            wait(20)
    except KeyboardInterrupt:
        behavior.on_stop(hw)
        hw.stop()

if __name__ == "__main__":
    main()
```

---

## Testing Different Configurations

### Test Matrix

| Behavior | Regulator | Kp | Ki | Kd | Use Case |
|----------|-----------|----|----|-----|----------|
| Lane | PID | 1.2 | 0.3 | 0.05 | Default smooth |
| Lane | PID | 1.5 | 0.4 | 0.08 | Faster response |
| Lane | PIFuzzy | 1.2 | 0.3 | 0.0 | Adaptive |
| Maze | PID | 2.0 | 0.4 | 0.1 | Default quick |
| Maze | PID | 1.8 | 0.3 | 0.15 | More damped |
| Maze | PIFuzzy | 2.0 | 0.4 | 0.0 | Adaptive |

---

## Troubleshooting

### Error: "TypeError: function takes X positional arguments but Y were given"

**Cause**: Missing required parameters (kp, ki, kd)

**Solution**: Always provide all three PID parameters:
```python
behavior = LaneKeeping(
    regulator_kp=1.2,  # Must provide
    regulator_ki=0.3,  # Must provide
    regulator_kd=0.05  # Must provide
)
```

### Regulator not being used

**Check**: Verify regulator_type is either "PID" or "PIFuzzy"
```python
# Correct
regulator_type="PID"

# Incorrect (will disable regulator)
regulator_type="pid"      # Wrong case
regulator_type="None"     # String "None"
regulator_type=None       # Will cause error
```

### Different behaviors need different tuning

**Reason**: LaneKeeping and MazeSolver have different control requirements

**Solution**: Use the recommended defaults as starting points, then fine-tune based on robot performance

---

## Next Steps

1. Test default parameters on your robot
2. Adjust one parameter at a time
3. Compare PID vs PI-Fuzzy performance
4. Log regulator state for analysis
5. Export data for ML training with regulator parameters
