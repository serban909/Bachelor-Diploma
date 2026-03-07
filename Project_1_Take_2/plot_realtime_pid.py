#!/usr/bin/env python3
"""
Real-time PID Plotting Script for EV3 Lane Keeping
Receives error and output values from EV3 over Wi-Fi and plots them in real-time.

Usage:
1. Update PC_IP in LaneKeeping_PID.py to match your PC's IP address
2. Run this script on your PC: python plot_realtime_pid.py
3. Run LaneKeeping_PID.py on your EV3
4. Watch the real-time plot!
"""

import socket
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import sys

# Configuration
LISTEN_PORT = 5005       # Must match PC_PORT in LaneKeeping_PID.py
MAX_POINTS = 500         # Maximum number of points to display on plot
BUFFER_SIZE = 1024       # UDP receive buffer size

# Data storage
steps = deque(maxlen=MAX_POINTS)
errors = deque(maxlen=MAX_POINTS)
outputs = deque(maxlen=MAX_POINTS)

# Setup UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', LISTEN_PORT))  # Listen on all interfaces
sock.settimeout(0.01)  # Non-blocking with 10ms timeout

print("="*60)
print("Real-time PID Plotter - Waiting for data from EV3")
print("="*60)
print(f"Listening on port {LISTEN_PORT}")
print("Press Ctrl+C to stop")
print("="*60)

# Setup the plot
fig, ax = plt.subplots(figsize=(12, 6))
line_error, = ax.plot([], [], 'r-', label='Error', linewidth=2)
line_output, = ax.plot([], [], 'b-', label='PID Output (Turn Rate)', linewidth=2)

ax.set_xlabel('Step', fontsize=12, fontweight='bold')
ax.set_ylabel('Value', fontsize=12, fontweight='bold')
ax.set_title('Real-time PID Controller - Lane Keeping', fontsize=14, fontweight='bold')
ax.legend(loc='upper right', fontsize=10)
ax.grid(True, alpha=0.3, linestyle='--')

# Set initial limits
ax.set_xlim(0, 100)
ax.set_ylim(-250, 250)

data_received = False

def receive_data():
    """Receive and parse data from EV3"""
    global data_received
    try:
        data, addr = sock.recvfrom(BUFFER_SIZE)
        message = data.decode().strip()
        
        # Parse CSV format: step,error,output
        parts = message.split(',')
        if len(parts) == 3:
            step = int(parts[0])
            error = float(parts[1])
            output = float(parts[2])
            
            steps.append(step)
            errors.append(error)
            outputs.append(output)
            
            if not data_received:
                print(f"✓ Connected! Receiving data from {addr[0]}")
                data_received = True
            
            return True
    except socket.timeout:
        pass
    except Exception as e:
        print(f"Error parsing data: {e}")
    
    return False

def update_plot(frame):
    """Update the plot with new data"""
    # Receive multiple packets per frame to reduce lag
    for _ in range(10):
        receive_data()
    
    if len(steps) > 0:
        # Update data
        line_error.set_data(steps, errors)
        line_output.set_data(steps, outputs)
        
        # Auto-scale x-axis
        if len(steps) > 10:
            min_step = min(steps)
            max_step = max(steps)
            margin = (max_step - min_step) * 0.1
            ax.set_xlim(min_step - margin, max_step + margin)
        
        # Auto-scale y-axis with some headroom
        if len(errors) > 0 and len(outputs) > 0:
            all_values = list(errors) + list(outputs)
            min_val = min(all_values)
            max_val = max(all_values)
            margin = max(abs(max_val), abs(min_val)) * 0.2
            ax.set_ylim(min_val - margin, max_val + margin)
    
    return line_error, line_output

# Create animation
ani = animation.FuncAnimation(
    fig, 
    update_plot, 
    interval=50,  # Update every 50ms
    blit=True,
    cache_frame_data=False
)

try:
    plt.tight_layout()
    plt.show()
except KeyboardInterrupt:
    print("\n" + "="*60)
    print("Plotting stopped by user")
    print("="*60)
finally:
    sock.close()
    if data_received:
        print(f"\nTotal points received: {len(steps)}")
        if len(errors) > 0:
            print(f"Error range: {min(errors):.2f} to {max(errors):.2f}")
        if len(outputs) > 0:
            print(f"Output range: {min(outputs):.2f} to {max(outputs):.2f}")
