# Practical Application Specification (Bachelor)

## 1. Project Deliverable: Source Code Repository

Public repository link:

- https://github.com/serban909/Bachelor-Diploma

Repository requirements:

- Visibility: **Public**
- Content: entire application source code
- Excluded content: compiled binaries / generated artifacts

Recommended exclusions in `.gitignore`:

- `__pycache__/`
- `*.pyc`
- `.venv/`
- build/output folders (if added later)

---

## 2. Application Build Steps

This project is Python-based and does not require a compilation step.

### 2.1. Prerequisites

- Windows 10/11 (or Linux/macOS with equivalent commands)
- Python 3.10+ installed
- `pip` package manager
- OpenSSH client tools (`ssh`, `scp`) available in terminal
- EV3 brick reachable over network (for robot deployment/run)

### 2.2. Build/Environment Preparation

1. Clone the repository:

```bash
git clone https://github.com/serban909/Bachelor-Diploma.git
cd Bachelor-Diploma
```

2. Create and activate a virtual environment:
```bash
python -m venv .venv

# For Windows PowerShell:
.\.venv\Scripts\Activate.ps1

# For Windows Command Prompt (CMD):
.\.venv\Scripts\activate.bat

# For Linux / macOS / Git Bash:
source .venv/bin/activate
```

3. Install required Python dependencies:

```bash
pip install matplotlib
```

Notes:

- `tkinter` is used by the UI and is usually included in standard Python installations.
- No executable/binary build is required for the desktop launcher.

---

## 3. Application Installation and Launch Steps

The application has two sides:

- PC Launcher UI (`ui_launcher.py`)
- EV3 robot control script (`fuzzy_robot.py`)

### 3.1. PC Side (Launcher)

1. Open a terminal in the project folder.
2. Activate the virtual environment:

```bash
# For Windows PowerShell:
.\.venv\Scripts\Activate.ps1

# For Windows Command Prompt (CMD):
.\.venv\Scripts\activate.bat

# For Linux / macOS / Git Bash:
source .venv/bin/activate
```

3. Start the launcher:

```bash
python ui_launcher.py
```

4. In the launcher window:

- Select Task (`LaneKeeping` or `MazeSolver`)
- Select Algorithm (`PID`, `FuzzyPI`, or `FuzzyRuleBased`)
- Set EV3 host (default: `ev3dev.local`)
- (Optional) enable on the fly maze map for maze mode
- Press **Start Robot**

### 3.2. EV3 Side (Deployment and Execution)

When **Start Robot** is pressed, the launcher:

1. Updates runtime parameters in `fuzzy_robot.py` (task, algorithm, PC IP)
2. Uploads script to EV3 via `scp`
3. Starts robot execution remotely via `ssh`:

```bash
brickrun -r -- pybricks-micropython /home/robot/fuzzy_robot.py
```

### 3.3. Runtime Behavior

- Robot streams telemetry over UDP (default port `5005`) to the PC.
- Launcher displays live plots:
  - Disturbance
  - Controller output (turn rate or speed)
- In Maze mode, optional on the fly maze-map visualization is available.

---

## 4. Verification Checklist

- Repository is publicly accessible.
- Source code is present in repository.
- Compiled binaries are not committed.
- `ui_launcher.py` starts successfully on PC.
- EV3 connection works (`ssh`/`scp`).
- Telemetry is received and plotted in on the fly.

---

## 5. Deliverable Filename

Uploaded document name:

**Albert-Tamasdan_Francisc-Serban_Practical_Application_CTIEN_Bachelor.md**
