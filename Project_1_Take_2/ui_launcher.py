import tkinter 
import socket
import threading
import re
import subprocess
from pathlib import Path
from collections import deque

import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot 
import matplotlib.animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Rectangle
from matplotlib.lines import Line2D
from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk

LISTEN_PORT=5005
MAX_POINTS=500
BUFFER_SIZE=1024

TASK_NAMES: dict[str, str]={
    "LaneKeeping": "Lane Keeping",
    "MazeSolver": "Maze Solver",
}

ALGORITHM_NAMES: dict[str, str]={
    "PID": "PID",
    "FuzzyPI": "Fuzzy PI",
    "FuzzyRuleBased": "Fuzzy Rule-Based",
}

def makeLabels(task: str, algorithm: str)->tuple[str, str, str]:
    taskName=TASK_NAMES.get(task, task)
    algorithmName=ALGORITHM_NAMES.get(algorithm, algorithm)
    title=f"{taskName} - {algorithmName}"
    
    if task == "LaneKeeping":
        inLabel="Disturbance (sensor difference)"
        outLabel=f"{algorithmName} output (turn rate deg/s)"
    else:
        inLabel="Disturbance (target - measured wall distance mm)"
        outLabel=f"{algorithmName} output (speed mm/s)"
        
    return title, inLabel, outLabel

COLOUR_BACKGROUND  = "#1e1e2e"
COLOUR_SURFACE     = "#2a2a3e"
COLOUR_BORDER      = "#44475a"
COLOUR_TEXT        = "#cdd6f4"
COLOUR_SUBTEXT     = "#a6adc8"
COLOUR_BUTTON_ACTIVE  = "#89b4fa"   
COLOUR_BUTTON_IDLE    = "#313244"   
COLOUR_GREEN       = "#a6e3a1"
COLOUR_YELLOW      = "#f9e2af"
COLOUR_RED         = "#f38ba8"

class DataReceiver:
    def __init__(self, port:int, maxPoints:int):
        self.steps=deque(maxlen=maxPoints)
        self.disturbances=deque(maxlen=maxPoints)
        self.outputs=deque(maxlen=maxPoints)
        self.cells={}
        self.currentPosition=(0, 0)
        
        self.port=port
        self.socket=None
        self.thread=None
        self.running=False
        self.connected=False
        self._onConnect=None
        self._onMeta=None

    def onConnect(self, callback):
        self._onConnect=callback

    def onMeta(self, callback):
        self._onMeta=callback
        
    def start(self):
        try: 
            self.socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind(("0.0.0.0", self.port))
            self.socket.settimeout(0.1)
        except OSError:
            raise RuntimeError (f"Cannot bind port {self.port}")
        
        self.running=True
        self.thread=threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
        
    def stop(self):
        self.running=False
        if self.socket:
            self.socket.close()
            self.socket=None
            
    def clear(self):
        self.steps.clear()
        self.disturbances.clear()
        self.outputs.clear()
        self.connected=False
        self.cells.clear()
        self.currentPosition=(0, 0)
        
    def loop(self):
        while self.running:
            try:
                raw, address=self.socket.recvfrom(BUFFER_SIZE)
                message=raw.decode().strip()
                
                if message.startswith("META,"):
                    _, label=message.split(",", 1)
                    if self._onMeta:
                        self._onMeta(label.strip())
                    continue

                if message.startswith("CELL,"):
                    parts=message.split(",")
                    if len(parts)==4:
                        x, y, walls = int(parts[1]), int(parts[2]), int(parts[3])
                        self.cells[(x, y)] = walls
                        self.currentPosition = (x, y)
                        if not self.connected:
                            self.connected=True
                            if self._onConnect:
                                self._onConnect(address[0])
                    continue

                parts=message.split(",")
                if len(parts)==3:
                    self.steps.append(int(parts[0]))
                    self.disturbances.append(float(parts[1]))
                    self.outputs.append(float(parts[2]))

                    if not self.connected:
                        self.connected=True
                        if self._onConnect:
                            self._onConnect(address[0])
                            
            except socket.timeout:
                pass
            except (OSError, ValueError):
                pass
            
class GraphWindow(tkinter.Toplevel):
    def __init__(self, parent, receiver: DataReceiver, task: str, algorithm: str, onClose=None):
        super().__init__(parent)
        self.configure(background=COLOUR_BACKGROUND)
        self.geometry("950x580")
        self.resizable(True, True)
        self.receiver=receiver
        self.ani=None
        self.onCloseCallBack=onClose
        
        self.buildFigure(task, algorithm)
        self.buildStatusBar()
        self.startAnimation()
        
        self.protocol("WM_DELETE_WINDOW", self.onClose)
        
    def buildFigure(self, task: str, algorithm: str):
        title, inLabel, outLabel=makeLabels(task, algorithm)
        
        self.title(title)
        
        self.figure, self.axis=matplotlib.pyplot.subplots(figsize=(10, 5))
        self.figure.patch.set_facecolor("#1e1e2e")
        self.axis.set_facecolor("#2a2a3e")
        
        self.lineDisturbance, = self.axis.plot(
            [], [], color=COLOUR_RED, linewidth=2, label=inLabel
        )
        
        self.lineOutput, = self.axis.plot(
            [], [], color=COLOUR_BUTTON_ACTIVE, linewidth=2, label=outLabel
        )
        
        self.axis.set_xlabel("Step", color=COLOUR_TEXT, fontsize=11, fontweight="bold")
        self.axis.set_ylabel("Value", color=COLOUR_TEXT, fontsize=11, fontweight="bold")
        self.axis.set_title(title, color=COLOUR_TEXT, fontsize=13, fontweight="bold")
        self.axis.tick_params(colors=COLOUR_SUBTEXT)
        self.axis.spines[:].set_color(COLOUR_BORDER)
        self.axis.set_xlim(0, 100)
        self.axis.set_ylim(-250, 250)
        self.axis.grid(True, alpha=0.2, linestyle="--", color=COLOUR_BORDER)

        self.axis.legend(
            loc="upper right", fontsize=9,
            facecolor=COLOUR_SURFACE, edgecolor=COLOUR_BORDER,
            labelcolor=COLOUR_TEXT,
        )

        self.canvas = FigureCanvasTkAgg(self.figure, master=self)
        self.canvas.get_tk_widget().pack(fill=tkinter.BOTH, expand=True, padx=8, pady=8)

        toolbar = NavigationToolbar2Tk(self.canvas, self)
        toolbar.update()
        toolbar.pack(side=tkinter.BOTTOM, fill=tkinter.X)

    def buildStatusBar(self):
        bar=tkinter.Frame(self, bg=COLOUR_BORDER, height=30)
        bar.pack(fill=tkinter.X, side=tkinter.BOTTOM)
        
        self.statusDot=tkinter.Label(bar, text="●", fg=COLOUR_YELLOW, bg=COLOUR_SURFACE, font=("Segoe UI", 10))
        self.statusDot.pack(side=tkinter.LEFT, padx=(8, 2))
        
        self.statusLabel=tkinter.Label(bar, text="Waiting for robot connection...", fg=COLOUR_SUBTEXT, bg=COLOUR_SURFACE, font=("Segoe UI", 9))
        self.statusLabel.pack(side=tkinter.LEFT)
        self.pointsLabel=tkinter.Label(bar, text="", fg=COLOUR_SUBTEXT, bg=COLOUR_SURFACE, font=("Segoe UI", 9))
        self.pointsLabel.pack(side=tkinter.RIGHT, padx=8)    
        
    def updateLabels(self, task: str, algorithm: str):
        title, inLabel, outLabel=makeLabels(task, algorithm)
        self.title(title)
        self.axis.set_title(title, color=COLOUR_TEXT, fontsize=13, fontweight="bold")
        self.lineDisturbance.set_label(inLabel)
        self.lineOutput.set_label(outLabel)
        self.axis.legend(
            loc="upper right", fontsize=9,
            facecolor=COLOUR_SURFACE, edgecolor=COLOUR_BORDER,
            labelcolor=COLOUR_TEXT,
        )
    
    def setStatusConnected(self, ip: str):
        self.statusDot.config(fg=COLOUR_GREEN)
        self.statusLabel.config(text=f"Connected to robot at {ip}")
        
    def startAnimation(self):
        self.animation = matplotlib.animation.FuncAnimation(
            self.figure,
            self.updateFrame,
            interval=50,         
            blit=True,
            cache_frame_data=False,
        )
        self.canvas.draw()
    
    def updateFrame(self, _frame):
        steps=list(self.receiver.steps)
        disturbances=list(self.receiver.disturbances)
        outputs=list(self.receiver.outputs)
        
        if not steps:
            return self.lineDisturbance, self.lineOutput
        
        self.lineDisturbance.set_data(steps, disturbances)
        self.lineOutput.set_data(steps, outputs)
        
        sMin, sMax = min(steps), max(steps)
        xMargin = max(1, (sMax - sMin) * 0.05)
        self.axis.set_xlim(sMin - xMargin, sMax + xMargin)
        
        if disturbances and outputs:
            peak=max(abs(value) for value in disturbances+outputs)
            yMargin=max(10, peak * 0.2)
            self.axis.set_ylim(-peak - yMargin, peak + yMargin)
        
        self.pointsLabel.config(text=f"Data points: {len(steps)}")
        
        return self.lineDisturbance, self.lineOutput
    
    def onClose(self):
        if self.animation:
            self.animation.event_source.stop()
        matplotlib.pyplot.close(self.figure)
        
        if self.onCloseCallBack:
            self.onCloseCallBack()
        
        self.destroy()
    
class MazeMapWindow(tkinter.Toplevel):
    def __init__(self, parent, receiver: DataReceiver, onClose=None):
        super().__init__(parent)
        self.title("Live Maze Map")
        self.configure(background=COLOUR_BACKGROUND)
        self.geometry("640x640")
        self.resizable(True, True)
        self.receiver=receiver
        self.animation=None
        self.onCloseCallBack=onClose
        
        self.buildFigure()
        self.buildStatusBar()
        self.startAnimation()
        
        self.protocol("WM_DELETE_WINDOW", self.onClose)
        
    def buildFigure(self):
        self.figure, self.axis=matplotlib.pyplot.subplots(figsize=(6, 6))
        self.figure.patch.set_facecolor(COLOUR_BACKGROUND)
        self.axis.set_facecolor(COLOUR_SURFACE)
        self.axis.set_aspect("equal")
        self.axis.set_title("Maze Map - DFS", color=COLOUR_TEXT, fontsize=13, fontweight="bold")
        self.axis.set_xlabel("X", color=COLOUR_TEXT, fontsize=10)
        self.axis.set_ylabel("Y", color=COLOUR_TEXT, fontsize=10)
        self.axis.tick_params(colors=COLOUR_SUBTEXT)
        
        for spine in self.axis.spines.values():
            spine.set_color(COLOUR_BORDER)
            
        self.canvas = FigureCanvasTkAgg(self.figure, master=self)
        self.canvas.get_tk_widget().pack(fill=tkinter.BOTH, expand=True, padx=8, pady=8)
        
    def buildStatusBar(self):
        bar=tkinter.Frame(self, bg=COLOUR_BORDER, height=30)
        bar.pack(fill=tkinter.X, side=tkinter.BOTTOM)
        
        self.cellsLabel=tkinter.Label(bar, text="Cells: 0", fg=COLOUR_SUBTEXT, bg=COLOUR_SURFACE, font=("Segoe UI", 9))
        self.cellsLabel.pack(side=tkinter.RIGHT, padx=8)
        
        self.posLabel=tkinter.Label(bar, text="Position: (0, 0)", fg=COLOUR_SUBTEXT, bg=COLOUR_SURFACE, font=("Segoe UI", 9))
        self.posLabel.pack(side=tkinter.LEFT, padx=8)
    
    def drawMazeCell(self, x, y, walls, isCurrent, isStart):
        fill=COLOUR_BUTTON_ACTIVE if isCurrent else (COLOUR_GREEN if isStart else COLOUR_SURFACE)
        self.axis.add_patch(Rectangle((x, y), 1, 1, facecolor=fill, edgecolor=COLOUR_BORDER, linewidth=1, zorder=1))
        textColor=COLOUR_BACKGROUND if (isCurrent or isStart) else COLOUR_TEXT
        self.axis.text(x+0.5, y+0.5, f"({x},{y})", color=textColor, fontsize=6, ha="center", va="center", zorder=3)
        
        W=COLOUR_TEXT
        lw=2
        
        if not (walls & 1):  
            self.axis.add_line(Line2D([x, x+1], [y+1, y+1], color=W, linewidth=lw, zorder=2))
        if not (walls & 2):  
            self.axis.add_line(Line2D([x+1, x+1], [y, y+1], color=W, linewidth=lw, zorder=2))
        if not (walls & 4):  
            self.axis.add_line(Line2D([x, x+1], [y, y], color=W, linewidth=lw, zorder=2))
        if not (walls & 8):  
            self.axis.add_line(Line2D([x, x], [y, y+1], color=W, linewidth=lw, zorder=2))
    
    def updateFrameMaze(self, _frame):
        cells=dict(self.receiver.cells)
        current=self.receiver.currentPosition
        
        if not cells:
            return
        
        self.axis.cla()
        self.axis.set_facecolor(COLOUR_SURFACE)
        self.axis.set_aspect("equal")
        self.axis.set_title("Maze Map - DFS", color=COLOUR_TEXT, fontsize=13, fontweight="bold")
        self.axis.set_xlabel("X", color=COLOUR_TEXT, fontsize=10)
        self.axis.set_ylabel("Y", color=COLOUR_TEXT, fontsize=10)
        self.axis.tick_params(colors=COLOUR_SUBTEXT)
        
        for spine in self.axis.spines.values():
            spine.set_color(COLOUR_BORDER)
            
        for (x, y), walls in cells.items():
            self.drawMazeCell(x, y, walls, isCurrent=(current==(x,y)), isStart=(x==0 and y==0))
            
        xs=[cx for (cx, _) in cells]
        ys=[cy for (_, cy) in cells]
        margin=1.5
        
        self.axis.set_xlim(min(xs)-margin, max(xs)+margin+1)
        self.axis.set_ylim(min(ys)-margin, max(ys)+margin+1)
        self.cellsLabel.config(text=f"Cells: {len(cells)}")
        self.posLabel.config(text=f"Position: ({current[0]}, {current[1]})")
    
    def startAnimation(self):
        self.animation = matplotlib.animation.FuncAnimation(
            self.figure,
            self.updateFrameMaze,
            interval=300,         
            blit=False,
            cache_frame_data=False,
        )
        self.canvas.draw()
    
    def onClose(self):
        if self.animation:
            self.animation.event_source.stop()
        matplotlib.pyplot.close(self.figure)
        
        if self.onCloseCallBack:
            self.onCloseCallBack()
        
        self.destroy()    
    
class LauncherApp(tkinter.Tk):
    def __init__(self):
        super().__init__()
        self.title("EV3 Robot Controller")
        self.configure(background=COLOUR_BACKGROUND)
        self.resizable(True, False)
        
        self.task=tkinter.StringVar(value="LaneKeeping")
        self.algorithm=tkinter.StringVar(value="PID")
        self.ev3Host=tkinter.StringVar(value="ev3dev.local")
        self.showMazeMap=tkinter.BooleanVar(value=False)
        self.graphWindow=None
        self.mazeMapWindow=None
        
        self.receiver=DataReceiver(LISTEN_PORT, MAX_POINTS)
        self.receiver.onConnect(self.handleConnect)
        self.receiver.onMeta(self.handleMeta)
        
        try:
            self.receiver.start()
        except RuntimeError as e:
            self.bootError=str(e)
        else:
            self.bootError=None
            
        self.buildUI()    
        
        if self.bootError:
            self.setStatus(f"Port error: {self.bootError}", COLOUR_RED)
            
        self.protocol("WM_DELETE_WINDOW", self.onClose)
        
    def buildUI(self):
        outer=tkinter.Frame(self, bg=COLOUR_BACKGROUND, padx=24, pady=20)
        outer.pack(fill=tkinter.BOTH, expand=True)
        
        tkinter.Label(
            outer,
            text="EV3  Robot  Controller",
            foreground=COLOUR_BUTTON_ACTIVE,
            background=COLOUR_BACKGROUND,
            font=("Segoe UI", 18, "bold")
        ).pack(pady=(0, 4))
        
        tkinter.Label(
            outer,
            text="Select a task and algorithm, then start the robot to see live data visualization.",
            foreground=COLOUR_SUBTEXT,
            background=COLOUR_BACKGROUND,
            font=("Segoe UI", 9)
        ).pack(pady=(0, 18))
        
        self.buildToggleRow(
            outer, 
            label="Task:",
            variable=self.task,
            options=[("Lane Keeping", "LaneKeeping"), ("Maze Solver", "MazeSolver")]
        )
        
        tkinter.Frame(outer, height=12, bg=COLOUR_BACKGROUND).pack()  
        
        self.buildToggleRow(
            outer, 
            label="Algorithm:",
            variable=self.algorithm,
            options=[
                ("PID", "PID"), 
                ("Fuzzy PI", "FuzzyPI"), 
                ("Fuzzy Rule-Based", "FuzzyRuleBased"), 
            ]
        )
        
        tkinter.Frame(outer, height=16, bg=COLOUR_BACKGROUND).pack()
        
        self.mazeCheckFrame=tkinter.Frame(outer, bg=COLOUR_BACKGROUND)
        self.mazeCheckFrame.pack(fill=tkinter.X)
        
        tkinter.Label(
            self.mazeCheckFrame,
            text="Options:",
            foreground=COLOUR_TEXT,
            background=COLOUR_BACKGROUND,
            font=("Segoe UI", 10, "bold"),
            width=10,
            anchor="w"
        ).pack(side=tkinter.LEFT)
        
        self.mazeMapCheck=tkinter.Checkbutton(
            self.mazeCheckFrame,
            text="Show Live Maze Map",
            variable=self.showMazeMap,
            foreground=COLOUR_TEXT,
            background=COLOUR_BACKGROUND,
            activeforeground=COLOUR_TEXT,
            activebackground=COLOUR_BACKGROUND,
            selectcolor="#000000",
            font=("Segoe UI", 10),
            relief=tkinter.FLAT,
            cursor="hand2",
        )
        
        self.mazeMapCheck.pack(side=tkinter.LEFT, padx=4)
        
        tkinter.Frame(outer, height=16, bg=COLOUR_BACKGROUND).pack()
        
        hostRow=tkinter.Frame(outer, bg=COLOUR_BACKGROUND)
        hostRow.pack(fill=tkinter.X)
        
        tkinter.Label(
            hostRow,
            text="EV3 Host:",
            foreground=COLOUR_TEXT,
            background=COLOUR_BACKGROUND,
            font=("Segoe UI", 10, "bold"),
            width=10,
            anchor="w"
        ).pack(side=tkinter.LEFT)
        
        tkinter.Entry(
            hostRow,
            textvariable=self.ev3Host,
            foreground=COLOUR_TEXT,
            background=COLOUR_SURFACE,
            insertbackground=COLOUR_TEXT,
            font=("Segoe UI", 10),
            relief=tkinter.FLAT,
            width=24,
        ).pack(side=tkinter.LEFT, padx=(6, 0))
        
        tkinter.Frame(outer, height=16, bg=COLOUR_BACKGROUND).pack()

        self.configureLabel=tkinter.Label(
            outer,
            text=self.configureText(),
            foreground=COLOUR_SUBTEXT,
            background=COLOUR_BACKGROUND,
            font=("Segoe UI", 9),
        )
        self.configureLabel.pack()
        
        tkinter.Frame(outer, height=16, bg=COLOUR_BACKGROUND).pack()
        
        self.graphButton=tkinter.Button(
            outer,
            text="Start Robot",
            foreground=COLOUR_BACKGROUND,
            background=COLOUR_BUTTON_ACTIVE,
            activeforeground=COLOUR_BACKGROUND,
            activebackground="#03161f",
            font=("Segoe UI", 11, "bold"),
            relief=tkinter.FLAT,
            cursor="hand2",
            padx=16, pady=8,
            command=self.openGraph,
        )
        
        self.graphButton.pack()
        
        tkinter.Frame(outer, height=20, background=COLOUR_BACKGROUND).pack()
        
        statusFrame=tkinter.Frame(outer, background=COLOUR_SURFACE, padx=6, pady=10)
        statusFrame.pack(fill=tkinter.X)
        
        self.statusDot=tkinter.Label(
            statusFrame, 
            text="●", 
            foreground=COLOUR_YELLOW, 
            background=COLOUR_SURFACE, 
            font=("Segoe UI", 10)
        )
        
        self.statusDot.pack(side=tkinter.LEFT, padx=(0, 4))
        
        self.statusLabel=tkinter.Label(
            statusFrame, 
            text="Listening on port 5005 - waiting for EV3…", 
            foreground=COLOUR_SUBTEXT, 
            background=COLOUR_SURFACE, 
            font=("Segoe UI", 9)
        )
        
        self.statusLabel.pack(side=tkinter.LEFT)
        
        tkinter.Label(
            outer,
            text="Deploy  fuzzy_robot.py  to the EV3 and run it",
            foreground=COLOUR_BORDER,
            background=COLOUR_BACKGROUND,
            font=("Segoe UI", 8)
        ).pack(pady=(10, 0))
        
    def buildToggleRow(self, parent, label, variable, options):
        row=tkinter.Frame(parent, bg=COLOUR_BACKGROUND)
        row.pack(fill=tkinter.X)
        
        tkinter.Label(
            row,
            text=label,
            foreground=COLOUR_TEXT,
            background=COLOUR_BACKGROUND,
            font=("Segoe UI", 10, "bold"),
            width=10,
            anchor="w"
        ).pack(side=tkinter.LEFT)
        
        buttonFrame=tkinter.Frame(row, bg=COLOUR_BACKGROUND)
        buttonFrame.pack(side=tkinter.LEFT)
        
        buttons={}
        
        def makeSelect(value, buttons):
            def select():
                variable.set(value)
                for v, btn in buttons.items():
                    btn.config(
                        background=COLOUR_BUTTON_ACTIVE if v==value else COLOUR_BUTTON_IDLE,
                        foreground=COLOUR_BACKGROUND if v==value else COLOUR_SUBTEXT,
                    )
                    self.configureLabel.config(text=self.configureText())
            return select
        
        for display, value in options:
            button=tkinter.Button(
                buttonFrame,
                text=f"  {display}  ",
                activebackground=COLOUR_BACKGROUND,
                background=COLOUR_BUTTON_ACTIVE if variable.get()==value else COLOUR_BUTTON_IDLE,
                foreground=COLOUR_BACKGROUND if variable.get()==value else COLOUR_SUBTEXT,
                activeforeground=COLOUR_BACKGROUND,
                font=("Segoe UI", 10),
                relief=tkinter.FLAT,
                cursor="hand2",
                padx=6, pady=5,
                command=makeSelect(value, buttons)
            )
            button.pack(side=tkinter.LEFT, padx=4)
            buttons[value]=button
        
        for display, value in options:
            buttons[value].config(command=makeSelect(value, buttons))
            
    def configureText(self):
        title, _, _=makeLabels(self.task.get(), self.algorithm.get())
        return f"Active: {title}"
    
    def setStatus(self, text: str, colour: str=COLOUR_YELLOW):
        dotColour=COLOUR_GREEN if colour==COLOUR_GREEN else colour
        self.statusDot.config(foreground=dotColour)
        self.statusLabel.config(text=text, foreground=colour if colour!=COLOUR_GREEN else COLOUR_SUBTEXT)
        
    def openGraph(self):
        if self.graphWindow and tkinter.Toplevel.winfo_exists(self.graphWindow):
            self.graphWindow.lift()
            self.graphWindow.focus_force()
            return
        
        task=self.task.get()
        algorithm=self.algorithm.get()
        
        self.patchRobotFile(task, algorithm)
        self.receiver.clear()
        self.graphWindow=GraphWindow(
            self, 
            self.receiver, 
            task, 
            algorithm, 
            onClose=self.onGraphClose
        )
        
        if task == "MazeSolver" and self.showMazeMap.get():
            self.mazeMapWindow=MazeMapWindow(
                self,
                self.receiver,
                onClose=self.onMazeMapClose
            )
        
        self.launchEV3()
    
    def onGraphClose(self):
        self.graphWindow=None
        
        if self.mazeMapWindow and tkinter.Toplevel.winfo_exists(self.mazeMapWindow):
            self.mazeMapWindow.onClose()
            
        self.mazeMapWindow=None
        
        self.setStatus("Run finished — choose task/algorithm and click Start Robot", COLOUR_YELLOW)
    
    def onMazeMapClose(self):
        self.mazeMapWindow=None    
    
    def patchRobotFile(self, task: str, algorithm: str):
        robotPath=Path(__file__).with_name("fuzzy_robot.py")

        if not robotPath.exists():
            return

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("10.255.255.255", 1))
                pcIp=s.getsockname()[0]
        except Exception:
            pcIp="127.0.0.1"

        text=robotPath.read_text()
        text=re.sub(r'^TASK\s*=\s*"[^"]*"',      f'TASK = "{task}"',           text, flags=re.MULTILINE)
        text=re.sub(r'^ALGORITHM\s*=\s*"[^"]*"',  f'ALGORITHM = "{algorithm}"', text, flags=re.MULTILINE)
        text=re.sub(r'"PC_IP":\s*"[^"]*"',        f'"PC_IP": "{pcIp}"',         text)
        robotPath.write_text(text)
        
    def launchEV3(self):
        robotPath=Path(__file__).with_name("fuzzy_robot.py")
        host=self.ev3Host.get()
        self.setStatus("Connecting to EV3…", COLOUR_YELLOW)
        threading.Thread(
            target=self.launchEV3Thread, 
            args=(robotPath, host), 
            daemon=True
        ).start()
        
    def launchEV3Thread(self, robotPath: Path, host: str):
        remote="/home/robot/fuzzy_robot.py"
        psCommand=(
            f"scp '{robotPath}' robot@{host}:{remote}"
            f"; if ($?) {{ ssh robot@{host} brickrun -r -- pybricks-micropython {remote} }}"
        )
        
        subprocess.Popen(
            ["powershell", "-NoExit", "-Command", psCommand], 
            creationflags=subprocess.CREATE_NEW_CONSOLE
        )
        
        self.after(0, lambda: self.setStatus(
            f"Running on EV3 via SSH ({host})…", COLOUR_YELLOW
        ))
    
    def handleConnect(self, ip: str):
        def onMain():
            self.setStatus(f"Connected  -  receiving data from  {ip}", COLOUR_GREEN)
            if self.graphWindow and tkinter.Toplevel.winfo_exists(self.graphWindow):
                self.graphWindow.setStatusConnected(ip)
        self.after(0, onMain)
        
    def handleMeta(self, label: str):
        self.after(0, lambda: self.applyMeta(label))
        
    def applyMeta(self, label: str):
        if "_" in label:
            task, algorithm = label.split("_", 1)
        else:
            task, algorithm = self.task.get(), self.algorithm.get()
        if self.graphWindow and tkinter.Toplevel.winfo_exists(self.graphWindow):
            self.graphWindow.updateLabels(task, algorithm)
    
    def onClose(self):
        self.receiver.stop()
        self.destroy()
        
if __name__ == "__main__":
    app=LauncherApp()
    app.mainloop()
            