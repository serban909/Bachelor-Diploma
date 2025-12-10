# Maze Configuration Guide

## Problem Analysis & Solutions

### 🔴 Why Was the Robot Spinning in Circles?

**Root Cause:** The code was using the **forward-facing sensor** (Port S4) for **both** wall following AND obstacle detection:

```python
# OLD CODE (WRONG):
forward_distance = hardware_adapter.distance_mm()
left_wall_distance = forward_distance  # ❌ Using same sensor for two purposes!
```

**What happened:**
1. Robot started with no obstacle in front (distance > 80mm)
2. PID tried to maintain 100mm from "left wall" 
3. But it was actually measuring **forward distance**, not left wall!
4. PID kept steering trying to correct non-existent error
5. Result: **Spinning in place** without moving forward

**When you placed obstacle in front:**
- Forward distance dropped below 80mm
- Triggered emergency `_handle_obstacle()` 
- Correctly turned left (left-hand rule)
- This is why it worked when you blocked it manually!

---

## ✅ Solution Implemented

### Dual Sensor Configuration

Now uses **TWO separate ultrasonic sensors**:

| Sensor | Port | Direction | Purpose |
|--------|------|-----------|---------|
| **Forward Ultrasonic** | S4 | Front | Obstacle detection ahead |
| **Left Ultrasonic** | S1 | Left side | Wall-following distance |

```python
# NEW CODE (CORRECT):
forward_distance = hardware_adapter.distance_mm()          # S4 - Forward sensor
left_wall_distance = hardware_adapter.left_wall_distance_mm()  # S1 - Left sensor
```

Now the PID correctly:
- Tracks left wall distance (Port S1)
- Detects forward obstacles independently (Port S4)
- No more spinning!

---

## 📏 Distance Adjustments for Your Maze

Based on the photo, the maze has **narrow corridors** (~200-250mm wide). Original settings were too large.

### Before (Too Large)
```python
target_wall_distance = 100mm   # ❌ Too far - robot would hit right wall
base_speed = 120mm/s           # ❌ Too fast for narrow corridors
min_forward_distance = 80mm    # ❌ Too close - late reaction
```

### After (Optimized for Photo Maze)
```python
target_wall_distance = 60mm    # ✅ Safe distance from left wall
base_speed = 100mm/s           # ✅ Moderate speed for safety
min_forward_distance = 120mm   # ✅ Early obstacle detection
```

---

## Maze Dimensions Analysis (From Photo)

### Corridor Width
- **Estimated:** 200-250mm
- **Robot width:** ~150mm (typical EV3)
- **Available space:** 50-100mm margin

### Wall Following Math
```
Corridor width: 250mm
Robot width: 150mm
Available margin: 100mm

target_wall_distance = 60mm (from left wall)
→ Robot center: 60mm + 75mm = 135mm from left
→ Right side: 135mm + 75mm = 210mm from left
→ Clearance from right wall: 250mm - 210mm = 40mm ✅ Safe!
```

### Why 60mm Target Distance?

1. **Safe from left wall:** 60mm > robot radius (75mm would scrape)
2. **Avoids right wall:** Keeps robot ~40mm from right wall
3. **PID tracking range:** Large enough for accurate ultrasonic readings
4. **Narrow corridor optimal:** Balanced for 200-250mm corridors

---

## Hardware Setup Required

### Sensor Placement

```
         [Front View]
    
    ┌─────────────────┐
    │                 │
    │   🤖 EV3 Brick  │
    │                 │
    └─────────────────┘
          ↑       ↑
          │       │
    ┌─────┴───┐   │
  ← │  S1 US  │   │
    │  LEFT   │   │
    └─────────┘   │
              ┌───┴────┐
              │  S4 US │ →
              │ FORWARD│
              └────────┘
```

**Critical:** Left sensor (S1) must face **perpendicular** to robot direction (90° left)!

### Port Connections

| Component | Port | Notes |
|-----------|------|-------|
| Left Motor | A | Drive |
| Right Motor | C | Drive |
| **Left Ultrasonic** | **S1** | **NEW - Side facing** |
| Left Color | S2 | Line following |
| Right Color | S3 | Line following |
| Forward Ultrasonic | S4 | Obstacle detection |

---

## Testing Procedure

### Step 1: Test Sensors Separately

```python
# Test left wall sensor
from pybricks.ev3devices import UltrasonicSensor
from pybricks.parameters import Port

left_sensor = UltrasonicSensor(Port.S1)
print("Left wall:", left_sensor.distance(), "mm")

# Should read 60-250mm when facing wall
# Should read 2000+ when open space
```

### Step 2: Test Wall Following (Straight Corridor)

Place robot in straight corridor:
- Left wall present
- No obstacles ahead

**Expected behavior:**
- Robot moves forward at 100mm/s
- Maintains ~60mm from left wall
- Smooth path (no oscillation)

### Step 3: Test Obstacle Detection

Place obstacle 200mm ahead:

**Expected behavior:**
- Robot approaches at 100mm/s
- At 120mm distance: stops
- Turns left 90°
- Checks if clear
- Continues or tries right

---

## Troubleshooting

### Issue: Robot still spins

**Check:**
1. Left sensor (S1) properly connected?
2. Left sensor facing perpendicular (90° left)?
3. `left_wall_distance_mm()` returning valid values?

**Debug:**
```python
# Add to step():
print("Left:", left_wall_distance, "Forward:", forward_distance)
```

Expected:
- Left: 50-200mm (wall distance)
- Forward: varies (obstacle detection)

---

### Issue: Robot hits left wall

**Cause:** `target_wall_distance` too small

**Fix:**
```python
target_wall_distance = 80  # Increase from 60mm
```

---

### Issue: Robot hits right wall

**Cause:** `target_wall_distance` too large for narrow corridor

**Fix:**
```python
target_wall_distance = 50  # Decrease from 60mm
```

---

### Issue: Oscillates left-right

**Cause:** PID gains too aggressive

**Fix:**
```python
regulator_kp = 8.0   # Reduce from 10.0
regulator_kd = 0.05  # Increase from 0.02
```

---

### Issue: Crashes into obstacles

**Cause:** `min_forward_distance` too small or speed too high

**Fix:**
```python
min_forward_distance = 150  # Increase from 120mm
base_speed = 80             # Reduce from 100mm/s
```

---

## Distance Tuning Guide

### For Your Specific Maze

Measure actual corridor width:

```python
# Formula:
target_wall_distance = (corridor_width - robot_width) / 3

# Example:
# Corridor: 230mm
# Robot: 150mm
# Available: 80mm
# Target: 80 / 3 ≈ 27mm... too small!

# Better formula:
target_wall_distance = (corridor_width - robot_width) / 2 - 10mm

# Example:
# (230 - 150) / 2 - 10 = 40 - 10 = 30mm
# But 30mm might be too tight for ultrasonic noise

# Recommended: 50-70mm for 200-250mm corridors
```

### Measurement Tips

1. **Corridor width:** Measure at narrowest point
2. **Robot width:** Measure with sensors attached
3. **Start conservative:** Use larger target_wall_distance first
4. **Tune down:** Gradually reduce if too much margin

---

## Expected Performance (Your Maze)

### With Correct Setup

| Metric | Value |
|--------|-------|
| **Average speed** | 80-100mm/s |
| **Wall tracking accuracy** | ±10mm |
| **Corner turn time** | ~2 seconds |
| **Maze completion** | 30-60 seconds (depends on size) |

### Behavior in Photo Maze

Looking at your maze photo:
- **Straight sections:** Smooth forward movement at 100mm/s
- **Corners:** Stops at 120mm, turns left, continues
- **Dead ends:** Stops, checks left (blocked), checks right, turns around
- **No open left passages:** Won't take unnecessary left turns

---

## Advanced Tuning (Optional)

### Speed vs Corridor Width

Narrow corridors → Slower speed:

```python
# Very narrow (150-200mm)
base_speed = 70
target_wall_distance = 40

# Medium (200-250mm) ✅ Your maze
base_speed = 100
target_wall_distance = 60

# Wide (250-350mm)
base_speed = 130
target_wall_distance = 80
```

### PID Gains vs Speed

Faster speed → More aggressive PID:

```python
# Slow & smooth (70mm/s)
regulator_kp = 6.0
regulator_kd = 0.01

# Medium (100mm/s) ✅ Current
regulator_kp = 10.0
regulator_kd = 0.02

# Fast (130mm/s)
regulator_kp = 12.0
regulator_kd = 0.05
```

---

## Summary of Changes

### What Was Fixed

1. ✅ Added `left_ultrasonic_sensor` to Port S1
2. ✅ Added `left_wall_distance_mm()` method
3. ✅ Fixed `step()` to use separate sensors
4. ✅ Reduced `target_wall_distance`: 100mm → 60mm
5. ✅ Reduced `base_speed`: 120mm/s → 100mm/s
6. ✅ Increased `min_forward_distance`: 80mm → 120mm

### Why Robot Was Spinning

- ❌ Used forward sensor for wall following
- ❌ PID corrected for wrong distance
- ❌ No actual forward movement
- ✅ NOW: Proper dual-sensor setup

### Why 100mm Was Too Much

- Your maze corridors: ~200-250mm wide
- Robot width: ~150mm
- 100mm target → Only 50mm from right wall → Too risky
- 60mm target → ~90mm from right wall → Much safer

---

## Next Steps

1. **Connect left sensor to Port S1** (perpendicular to robot)
2. **Upload updated code** to EV3
3. **Test in straight corridor** first
4. **Adjust `target_wall_distance`** if needed (50-70mm range)
5. **Test full maze** with obstacles

Your robot should now:
- ✅ Move forward smoothly
- ✅ Follow left wall at 60mm
- ✅ Detect obstacles at 120mm
- ✅ Navigate your maze successfully!
