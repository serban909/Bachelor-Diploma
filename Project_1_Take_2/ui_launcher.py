#!/usr/bin/env python3
"""
EV3 Robot Controller — UI Launcher
====================================
Provides a graphical interface to:
  1. Choose Task      (Lane Keeping | Maze Solver)
  2. Choose Algorithm (PID          | Fuzzy)
  3. Open a live graph that auto-configures its labels from the EV3's
     META packet (same mechanism used by plot_realtime.py)

SOLID responsibilities
  DataReceiver  – background UDP thread, owns all socket logic (S)
  GraphWindow   – Toplevel with embedded matplotlib figure         (S)
  LauncherApp   – main window, wires components, owns UI state     (S)
  CONTROLLER_LABELS – open/closed label registry, extend without   (O)
                      touching any other class

Run:  python ui_launcher.py
Then deploy fuzzy_robot.py to the EV3 and run it.
"""

import tkinter as tk
from tkinter import ttk, font
import socket
import threading
import re
import subprocess
from pathlib import Path
from collections import deque

import matplotlib
matplotlib.use("TkAgg")                          # must be set before pyplot import
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ─────────────────────────────────────────────────────────────────────────────
# CONSTANTS
# ─────────────────────────────────────────────────────────────────────────────
LISTEN_PORT = 5005
MAX_POINTS  = 500
BUFFER_SIZE = 1024

# Human-readable display names – extend here without touching any other code
_TASK_NAMES: dict[str, str] = {
    "LaneKeeping": "Lane Keeping",
    "MazeSolver":  "Maze Solver",
}
_ALGO_NAMES: dict[str, str] = {
    "PID":          "PID",
    "FuzzyPI":      "Fuzzy PI",
    "AbsoluteFuzzy":"Absolute Fuzzy",
    "FuzzyAutomata":"Fuzzy Automata",
}

def _make_labels(task: str, algorithm: str) -> tuple[str, str, str]:
    """Build (title, error_label, output_label) from task + algorithm keys."""
    task_name = _TASK_NAMES.get(task, task)
    algo_name = _ALGO_NAMES.get(algorithm, algorithm)
    title = f"{task_name}  —  {algo_name}"
    if task == "LaneKeeping":
        err_lbl = "Error  (sensor difference)"
        out_lbl = f"{algo_name} output  (turn rate  deg/s)"
    else:
        err_lbl = "Error  (target − measured wall dist  mm)"
        out_lbl = f"{algo_name} output  (steering  deg/s)"
    return title, err_lbl, out_lbl

# Colour palette
CLR_BG          = "#1e1e2e"
CLR_SURFACE     = "#2a2a3e"
CLR_BORDER      = "#44475a"
CLR_TEXT        = "#cdd6f4"
CLR_SUBTEXT     = "#a6adc8"
CLR_BTN_ACTIVE  = "#89b4fa"   # blue – selected option
CLR_BTN_IDLE    = "#313244"   # unselected option
CLR_GREEN       = "#a6e3a1"
CLR_YELLOW      = "#f9e2af"
CLR_RED         = "#f38ba8"


# ─────────────────────────────────────────────────────────────────────────────
# DATA RECEIVER  (Single Responsibility: background UDP intake)
# ─────────────────────────────────────────────────────────────────────────────
class DataReceiver:
    """
    Listens on a UDP socket in a daemon thread.
    Fills thread-safe deques consumed by the graph window.
    Notifies observers via callbacks instead of sharing state directly.
    """

    def __init__(self, port: int, max_points: int):
        self.steps   = deque(maxlen=max_points)
        self.errors  = deque(maxlen=max_points)
        self.outputs = deque(maxlen=max_points)

        self._port          = port
        self._sock          = None
        self._thread        = None
        self._running       = False
        self._connected     = False  # reset by clear() so each run fires on_connect
        self._on_connect    = None   # callback(ip: str)
        self._on_meta       = None   # callback(label: str)

    # ── Observer registration ─────────────────────────────────────────────
    def on_connect(self, callback):
        """Register a callback fired once when the first data packet arrives."""
        self._on_connect = callback

    def on_meta(self, callback):
        """Register a callback fired when a META identification packet arrives."""
        self._on_meta = callback

    # ── Lifecycle ─────────────────────────────────────────────────────────
    def start(self):
        """Bind the socket and start the background thread."""
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.bind(("0.0.0.0", self._port))
            self._sock.settimeout(0.1)
        except OSError as exc:
            raise RuntimeError(f"Cannot bind port {self._port}: {exc}") from exc

        self._running = True
        self._thread  = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Signal the thread to stop and release the socket."""
        self._running = False
        if self._sock:
            self._sock.close()
            self._sock = None

    def clear(self):
        """Discard all buffered data and reset connection state for a new run."""
        self.steps.clear()
        self.errors.clear()
        self.outputs.clear()
        self._connected = False

    # ── Internal loop (daemon thread) ─────────────────────────────────────
    def _loop(self):
        while self._running:
            try:
                raw, addr = self._sock.recvfrom(BUFFER_SIZE)
                message   = raw.decode().strip()

                if message.startswith("META,"):
                    _, label = message.split(",", 1)
                    if self._on_meta:
                        self._on_meta(label.strip())
                    continue

                parts = message.split(",")
                if len(parts) == 3:
                    self.steps.append(int(parts[0]))
                    self.errors.append(float(parts[1]))
                    self.outputs.append(float(parts[2]))

                    if not self._connected:
                        self._connected = True
                        if self._on_connect:
                            self._on_connect(addr[0])

            except socket.timeout:
                pass
            except (OSError, ValueError):
                pass  # socket closed or malformed packet – keep going


# ─────────────────────────────────────────────────────────────────────────────
# GRAPH WINDOW  (Single Responsibility: matplotlib visualisation only)
# ─────────────────────────────────────────────────────────────────────────────
class GraphWindow(tk.Toplevel):
    """
    A Toplevel window that embeds a live matplotlib figure.
    Receives a DataReceiver it reads from – separation of concerns.
    Labels are set immediately from the UI selection; the META packet
    can also update them at runtime via update_labels().
    """

    def __init__(self, parent, receiver: DataReceiver, task: str, algorithm: str,
                 on_close=None):
        super().__init__(parent)
        self.title("Live Controller Graph")
        self.configure(bg=CLR_BG)
        self.geometry("950x580")
        self.resizable(True, True)

        self._receiver    = receiver
        self._ani         = None
        self._on_close_cb = on_close

        self._build_figure(task, algorithm)
        self._build_status_bar()
        self._start_animation()

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── Private builders ──────────────────────────────────────────────────
    def _build_figure(self, task: str, algorithm: str):
        title, err_lbl, out_lbl = _make_labels(task, algorithm)

        self._fig, self._ax = plt.subplots(figsize=(10, 5))
        self._fig.patch.set_facecolor("#1e1e2e")
        self._ax.set_facecolor("#2a2a3e")

        self._line_error,  = self._ax.plot(
            [], [], color="#f38ba8", linewidth=2, label=err_lbl
        )
        self._line_output, = self._ax.plot(
            [], [], color="#89b4fa", linewidth=2, label=out_lbl
        )

        self._ax.set_xlabel("Step",  color=CLR_TEXT, fontsize=11, fontweight="bold")
        self._ax.set_ylabel("Value", color=CLR_TEXT, fontsize=11, fontweight="bold")
        self._ax.set_title(title,    color=CLR_TEXT, fontsize=13, fontweight="bold")
        self._ax.tick_params(colors=CLR_SUBTEXT)
        self._ax.spines[:].set_color(CLR_BORDER)
        self._ax.set_xlim(0, 100)
        self._ax.set_ylim(-250, 250)
        self._ax.grid(True, alpha=0.2, linestyle="--", color=CLR_BORDER)

        legend = self._ax.legend(
            loc="upper right", fontsize=9,
            facecolor=CLR_SURFACE, edgecolor=CLR_BORDER,
            labelcolor=CLR_TEXT,
        )

        self._canvas = FigureCanvasTkAgg(self._fig, master=self)
        self._canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

    def _build_status_bar(self):
        bar = tk.Frame(self, bg=CLR_SURFACE, height=28)
        bar.pack(fill=tk.X, side=tk.BOTTOM)

        self._status_dot = tk.Label(
            bar, text="●", fg=CLR_YELLOW, bg=CLR_SURFACE, font=("Segoe UI", 10)
        )
        self._status_dot.pack(side=tk.LEFT, padx=(8, 2))

        self._status_lbl = tk.Label(
            bar, text="Waiting for EV3 data…",
            fg=CLR_SUBTEXT, bg=CLR_SURFACE, font=("Segoe UI", 9)
        )
        self._status_lbl.pack(side=tk.LEFT)

        self._points_lbl = tk.Label(
            bar, text="", fg=CLR_SUBTEXT, bg=CLR_SURFACE, font=("Segoe UI", 9)
        )
        self._points_lbl.pack(side=tk.RIGHT, padx=8)

    # ── Public API ────────────────────────────────────────────────────────
    def update_labels(self, task: str, algorithm: str):
        """Called by LauncherApp when a META packet arrives."""
        title, err_lbl, out_lbl = _make_labels(task, algorithm)
        self._ax.set_title(title, color=CLR_TEXT, fontsize=13, fontweight="bold")
        self._line_error.set_label(err_lbl)
        self._line_output.set_label(out_lbl)
        self._ax.legend(
            loc="upper right", fontsize=9,
            facecolor=CLR_SURFACE, edgecolor=CLR_BORDER,
            labelcolor=CLR_TEXT,
        )

    def set_status_connected(self, ip: str):
        self._status_dot.config(fg=CLR_GREEN)
        self._status_lbl.config(text=f"Connected  –  receiving from {ip}")

    # ── Animation ─────────────────────────────────────────────────────────
    def _start_animation(self):
        self._ani = animation.FuncAnimation(
            self._fig,
            self._update_frame,
            interval=50,          # 20 FPS
            blit=True,
            cache_frame_data=False,
        )
        self._canvas.draw()

    def _update_frame(self, _frame):
        # Snapshot the deques into plain lists before any iteration.
        # The UDP thread appends to the deques concurrently; passing a live
        # deque to Matplotlib's C internals risks "deque mutated during
        # iteration" RuntimeError.  A list() copy is instantaneous and safe.
        steps   = list(self._receiver.steps)
        errors  = list(self._receiver.errors)
        outputs = list(self._receiver.outputs)

        if not steps:
            return self._line_error, self._line_output

        self._line_error.set_data(steps, errors)
        self._line_output.set_data(steps, outputs)

        # Auto-scale x
        s_min, s_max = min(steps), max(steps)
        x_margin     = max(1, (s_max - s_min) * 0.05)
        self._ax.set_xlim(s_min - x_margin, s_max + x_margin)

        # Auto-scale y (symmetric)
        if errors and outputs:
            peak     = max(abs(v) for v in errors + outputs)
            y_margin = max(10, peak * 0.2)
            self._ax.set_ylim(-peak - y_margin, peak + y_margin)

        # Update point counter
        self._points_lbl.config(text=f"Points: {len(steps)}")

        return self._line_error, self._line_output

    def _on_close(self):
        if self._ani:
            self._ani.event_source.stop()
        plt.close(self._fig)
        if self._on_close_cb:
            self._on_close_cb()
        self.destroy()


# ─────────────────────────────────────────────────────────────────────────────
# LAUNCHER APP  (wires DataReceiver + GraphWindow, owns UI state)
# ─────────────────────────────────────────────────────────────────────────────
class LauncherApp(tk.Tk):
    """
    Main application window.
    Owns the UI state (task/algorithm selection) and the DataReceiver.
    Delegates all graph rendering to GraphWindow.
    """

    def __init__(self):
        super().__init__()
        self.title("EV3 Robot Controller")
        self.resizable(True, False)
        self.configure(bg=CLR_BG)

        # State
        self._task      = tk.StringVar(value="LaneKeeping")
        self._algorithm = tk.StringVar(value="PID")
        self._ev3_host  = tk.StringVar(value="ev3dev.local")
        self._graph_win = None

        # Start the shared data receiver immediately
        self._receiver = DataReceiver(LISTEN_PORT, MAX_POINTS)
        self._receiver.on_connect(self._handle_connect)
        self._receiver.on_meta(self._handle_meta)
        try:
            self._receiver.start()
        except RuntimeError as exc:
            self._boot_error = str(exc)
        else:
            self._boot_error = None

        self._build_ui()

        if self._boot_error:
            self._set_status(f"⚠  {self._boot_error}", CLR_RED)

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── UI construction ───────────────────────────────────────────────────
    def _build_ui(self):
        outer = tk.Frame(self, bg=CLR_BG, padx=24, pady=20)
        outer.pack(fill=tk.BOTH, expand=True)

        # Header
        tk.Label(
            outer,
            text="EV3  Robot  Controller",
            font=("Segoe UI", 18, "bold"),
            fg=CLR_BTN_ACTIVE, bg=CLR_BG,
        ).pack(pady=(0, 4))

        tk.Label(
            outer,
            text="Select task and algorithm, then open the live graph",
            font=("Segoe UI", 9),
            fg=CLR_SUBTEXT, bg=CLR_BG,
        ).pack(pady=(0, 18))

        self._build_toggle_row(
            outer,
            label="Task",
            variable=self._task,
            options=[("Lane Keeping", "LaneKeeping"), ("Maze Solver", "MazeSolver")],
        )

        tk.Frame(outer, bg=CLR_BG, height=12).pack()

        self._build_toggle_row(
            outer,
            label="Algorithm",
            variable=self._algorithm,
            options=[
                ("PID",          "PID"),
                ("Fuzzy PI",     "FuzzyPI"),
                ("Abs. Fuzzy",   "AbsoluteFuzzy"),
                ("Automata",     "FuzzyAutomata"),
            ],
        )

        tk.Frame(outer, bg=CLR_BG, height=16).pack()

        # EV3 hostname / IP entry
        host_row = tk.Frame(outer, bg=CLR_BG)
        host_row.pack(fill=tk.X)

        tk.Label(
            host_row, text="EV3 Host", width=10, anchor="w",
            font=("Segoe UI", 10, "bold"),
            fg=CLR_TEXT, bg=CLR_BG,
        ).pack(side=tk.LEFT)

        tk.Entry(
            host_row,
            textvariable=self._ev3_host,
            font=("Segoe UI", 10),
            bg=CLR_SURFACE, fg=CLR_TEXT,
            insertbackground=CLR_TEXT,
            relief=tk.FLAT,
            width=22,
        ).pack(side=tk.LEFT, padx=4, ipady=4)

        tk.Label(
            host_row,
            text="(SSH fallback if pybricksdev not found)",
            font=("Segoe UI", 8),
            fg=CLR_BORDER, bg=CLR_BG,
        ).pack(side=tk.LEFT, padx=(6, 0))

        tk.Frame(outer, bg=CLR_BG, height=16).pack()

        # Selected configuration preview
        self._config_lbl = tk.Label(
            outer,
            text=self._config_text(),
            font=("Segoe UI", 9, "italic"),
            fg=CLR_SUBTEXT, bg=CLR_BG,
        )
        self._config_lbl.pack()

        tk.Frame(outer, bg=CLR_BG, height=16).pack()

        # Graph button
        self._graph_btn = tk.Button(
            outer,
            text="  Open Live Graph  ",
            font=("Segoe UI", 11, "bold"),
            fg=CLR_BG, bg=CLR_BTN_ACTIVE,
            activebackground="#74c7ec",
            activeforeground=CLR_BG,
            relief=tk.FLAT,
            cursor="hand2",
            padx=16, pady=8,
            command=self._open_graph,
        )
        self._graph_btn.pack()

        tk.Frame(outer, bg=CLR_BG, height=20).pack()

        # Status bar
        status_frame = tk.Frame(outer, bg=CLR_SURFACE, pady=6, padx=10)
        status_frame.pack(fill=tk.X)

        self._status_dot = tk.Label(
            status_frame, text="●", fg=CLR_YELLOW,
            bg=CLR_SURFACE, font=("Segoe UI", 10)
        )
        self._status_dot.pack(side=tk.LEFT, padx=(0, 4))

        self._status_lbl = tk.Label(
            status_frame,
            text="Listening on port 5005 – waiting for EV3…",
            font=("Segoe UI", 9),
            fg=CLR_SUBTEXT, bg=CLR_SURFACE,
        )
        self._status_lbl.pack(side=tk.LEFT)

        # Hint
        tk.Label(
            outer,
            text="Deploy  fuzzy_robot.py  to the EV3 and run it",
            font=("Segoe UI", 8),
            fg=CLR_BORDER, bg=CLR_BG,
        ).pack(pady=(10, 0))

    def _build_toggle_row(self, parent, label: str, variable: tk.StringVar, options: list):
        """Build a labelled row of mutually-exclusive toggle buttons."""
        row = tk.Frame(parent, bg=CLR_BG)
        row.pack(fill=tk.X)

        tk.Label(
            row, text=label, width=10, anchor="w",
            font=("Segoe UI", 10, "bold"),
            fg=CLR_TEXT, bg=CLR_BG,
        ).pack(side=tk.LEFT)

        btn_frame = tk.Frame(row, bg=CLR_BG)
        btn_frame.pack(side=tk.LEFT)

        buttons = {}

        def make_select(value, btns):
            def select():
                variable.set(value)
                for v, b in btns.items():
                    b.config(
                        bg=CLR_BTN_ACTIVE if v == value else CLR_BTN_IDLE,
                        fg=CLR_BG        if v == value else CLR_SUBTEXT,
                    )
                self._config_lbl.config(text=self._config_text())
            return select

        for display, value in options:
            btn = tk.Button(
                btn_frame,
                text=f"  {display}  ",
                font=("Segoe UI", 10),
                bg=CLR_BTN_ACTIVE if variable.get() == value else CLR_BTN_IDLE,
                fg=CLR_BG         if variable.get() == value else CLR_SUBTEXT,
                activebackground=CLR_BTN_ACTIVE,
                activeforeground=CLR_BG,
                relief=tk.FLAT,
                cursor="hand2",
                padx=6, pady=5,
            )
            btn.pack(side=tk.LEFT, padx=4)
            buttons[value] = btn

        # Wire commands now that all buttons exist
        for display, value in options:
            buttons[value].config(command=make_select(value, buttons))

    # ── Helpers ───────────────────────────────────────────────────────────
    def _config_text(self):
        title, _, _ = _make_labels(self._task.get(), self._algorithm.get())
        return f"Active: {title}"

    def _set_status(self, text: str, colour: str = CLR_YELLOW):
        dot_colour = CLR_GREEN if colour == CLR_GREEN else colour
        self._status_dot.config(fg=dot_colour)
        self._status_lbl.config(text=text, fg=colour if colour != CLR_GREEN else CLR_SUBTEXT)

    # ── Event handlers ────────────────────────────────────────────────────
    def _open_graph(self):
        """Patch fuzzy_robot.py, launch it on the EV3, and open the graph."""
        if self._graph_win and tk.Toplevel.winfo_exists(self._graph_win):
            self._graph_win.lift()
            self._graph_win.focus_force()
            return

        task      = self._task.get()
        algorithm = self._algorithm.get()

        self._patch_robot_file(task, algorithm)
        self._receiver.clear()
        self._graph_win = GraphWindow(
            self, self._receiver, task, algorithm,
            on_close=self._on_graph_closed,
        )
        self._launch_ev3()

    def _on_graph_closed(self):
        """Called when the GraphWindow is closed; resets state for the next run."""
        self._graph_win = None
        self._set_status("Run finished — choose task/algorithm and click Open Live Graph", CLR_YELLOW)

    def _patch_robot_file(self, task: str, algorithm: str):
        """Write the selected TASK and ALGORITHM into fuzzy_robot.py."""
        robot_path = Path(__file__).with_name("fuzzy_robot.py")
        if not robot_path.exists():
            return
        text = robot_path.read_text()
        text = re.sub(r'^TASK\s*=\s*"[^"]*"',
                      f'TASK = "{task}"',      text, flags=re.MULTILINE)
        text = re.sub(r'^ALGORITHM\s*=\s*"[^"]*"',
                      f'ALGORITHM = "{algorithm}"', text, flags=re.MULTILINE)
        robot_path.write_text(text)

    def _launch_ev3(self):
        """Kick off the EV3 launch in a background thread to keep the UI responsive."""
        robot_path = Path(__file__).with_name("fuzzy_robot.py")
        host       = "10.194.244.43"
        self._set_status("Connecting to EV3…", CLR_YELLOW)
        threading.Thread(
            target=self._launch_ev3_worker,
            args=(robot_path, host),
            daemon=True,
        ).start()

    def _launch_ev3_worker(self, robot_path: Path, host: str):
        """Background worker: tries pybricksdev first, then SSH/SCP."""
        # ── Attempt 1: pybricksdev (Bluetooth LE) ────────────────────────
        try:
            subprocess.Popen(
                ["pybricksdev", "run", "ble", str(robot_path)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.after(0, lambda: self._set_status(
                "Launching on EV3 via Bluetooth…", CLR_YELLOW))
            return
        except FileNotFoundError:
            pass  # pybricksdev not installed – fall through to SSH

        # ── Attempt 2: SCP + SSH in an interactive PowerShell window ────
        # Both commands run inside a new PowerShell window so the user can
        # type the password if SSH keys are not set up (default: maker).
        remote = "/home/robot/fuzzy_robot.py"
        ps_cmd = (
            f"scp '{robot_path}' robot@{host}:{remote}"
            f"; if ($?) {{ ssh robot@{host} brickrun -r -- pybricks-micropython {remote} }}"
        )
        subprocess.Popen(
            ["powershell", "-NoExit", "-Command", ps_cmd],
            creationflags=subprocess.CREATE_NEW_CONSOLE,
        )

        self.after(0, lambda: self._set_status(
            f"Running on EV3 via SSH ({host})…", CLR_YELLOW))

    def _handle_connect(self, ip: str):
        """Called from receiver thread – schedule UI update on main thread.
        NOTE: winfo_exists and all Tk calls must run on the main thread;
        wrapping everything in a single after() closure guarantees that.
        """
        def _on_main():
            self._set_status(f"Connected  –  receiving data from  {ip}", CLR_GREEN)
            if self._graph_win and tk.Toplevel.winfo_exists(self._graph_win):
                self._graph_win.set_status_connected(ip)
        self.after(0, _on_main)

    def _handle_meta(self, label: str):
        """Called from receiver thread – update labels on main thread."""
        self.after(0, lambda: self._apply_meta(label))

    def _apply_meta(self, label: str):
        # META label format from EV3: "LaneKeeping-AbsoluteFuzzy"
        if "-" in label:
            task, algorithm = label.split("-", 1)
        else:
            task, algorithm = self._task.get(), self._algorithm.get()
        if self._graph_win and tk.Toplevel.winfo_exists(self._graph_win):
            self._graph_win.update_labels(task, algorithm)

    def _on_close(self):
        self._receiver.stop()
        self.destroy()


# ─────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = LauncherApp()
    app.mainloop()
