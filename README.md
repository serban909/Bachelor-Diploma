# EV3 Robot Navigation System

## Architecture Overview

This project follows **SOLID principles** with clear separation of concerns:

### Core Classes

#### 1. **Behavior** (Abstract Base Class)
- Defines interface for all robot behaviors
- Methods: `on_start()`, `step()`, `on_stop()`

#### 2. **HardwareAdapter**
- Encapsulates all hardware interactions
- Provides abstraction over motors, sensors, and drive base
- Single responsibility: Hardware management

#### 3. **PathLogger**
- **Single Responsibility**: Logging decisions and actions
- Tracks:
  - Decisions made by the robot
  - Turn actions (angle and description)
  - Forward movements (distance)
  - Sensor readings (value and interpretation)
- Can be used by ANY behavior (MazeSolver, LaneKeeping, future behaviors)
- Outputs to console and provides data for export

#### 4. **PathMapper**
- **Single Responsibility**: Visual path representation
- Tracks robot position (x, y coordinates)
- Tracks robot orientation (0°=North, 90°=East, 180°=South, 270°=West)
- Creates ASCII map using:
  - `S` = Start position
  - `|` = Vertical movement
  - `-` = Horizontal movement
  - `*` = Current position
- Updates position based on movements and turns

#### 5. **DataExporter**
- **Single Responsibility**: Data persistence
- Exports logs to text files
- Exports maps to text files
- Exports ML training data in CSV format
- All methods are static (utility class)

### Behavior Implementations

#### **MazeSolver**
- Implements left-hand rule maze solving
- Uses `PathLogger` for decision tracking
- Uses `PathMapper` for visualization
- Exports data files on completion:
  - `maze_run_log.txt` - Human-readable log
  - `maze_run_map.txt` - ASCII map
  - `maze_training.csv` - ML-ready data

#### **LaneKeeping**
- Follows a line using color sensors
- Uses `PathLogger` for periodic sensor logging
- Exports data files on completion:
  - `lane_log.txt` - Sensor readings log
  - `lane_training.csv` - ML-ready data

#### **BehaviorFactory**
- Creates behavior instances based on string names
- Factory pattern for extensibility

### File Organization

```
Project_1/
├── main.py                 # Entry point
├── Behavior.py             # Abstract base class
├── HardwareAdapter.py      # Hardware abstraction
├── BehaviorFactory.py      # Behavior creation
│
├── MazeSolver.py           # Maze navigation behavior
├── LaneKeeping.py          # Line following behavior
│
├── PathLogger.py           # Decision/action logging (REUSABLE)
├── PathMapper.py           # ASCII map visualization (REUSABLE)
├── DataExporter.py         # File export utilities (REUSABLE)
│
└── README.md               # This file
```

### Data Export Files

When a behavior completes, it exports:

1. **Log File** (`*_log.txt`)
   - Decision entries
   - Turn actions
   - Movement records
   - Sensor readings

2. **Map File** (`*_map.txt`)
   - Final position and direction
   - ASCII visualization of path
   - Statistics (cells visited)

3. **Training Data** (`*_training.csv`)
   - CSV format for machine learning
   - Columns: decision_num, action_type, value1, value2, description
   - Ready for neural network training

### SOLID Principles Applied

✅ **Single Responsibility Principle**
- PathLogger: Only logging
- PathMapper: Only mapping
- DataExporter: Only file I/O
- HardwareAdapter: Only hardware control

✅ **Open/Closed Principle**
- New behaviors can be added without modifying existing code
- Extend Behavior class for new robot behaviors

✅ **Liskov Substitution Principle**
- Any Behavior implementation can be used interchangeably
- BehaviorFactory returns Behavior type

✅ **Interface Segregation Principle**
- Clean, focused interfaces
- Behaviors don't depend on unused methods

✅ **Dependency Inversion Principle**
- Behaviors depend on abstractions (HardwareAdapter, PathLogger, PathMapper)
- Not tied to specific implementations

### Future Extensions

1. **Machine Learning Integration**
   - Use exported CSV files for training
   - Neural network can learn from decision patterns
   - Predict optimal paths

2. **New Behaviors**
   - Create new classes extending `Behavior`
   - Reuse PathLogger and PathMapper
   - No changes to existing code needed

3. **Advanced Mapping**
   - Extend PathMapper for 3D visualization
   - Add obstacle markers
   - Track visited vs unvisited areas

4. **Real-time Analysis**
   - Stream data during execution
   - Live path visualization
   - Performance metrics dashboard

### Usage

```python
# Run maze solver
behavior = BehaviorFactory.create("maze", 
    wall_distance_mm=180, 
    forward_distance_mm=150, 
    probe_turn_deg=90)

# Run lane keeper
behavior = BehaviorFactory.create("lane",
    target_reflect=40,
    base_speed=140,
    kp=1.2,
    threshold=28)
```

### Export Locations

All data is exported to: `/home/robot/Project_1/`
- Maze: `maze_run_log.txt`, `maze_run_map.txt`, `maze_training.csv`
- Lane: `lane_log.txt`, `lane_training.csv`
