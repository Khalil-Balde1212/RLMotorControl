import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import time


class SerialPlotter:    
    """
    Real-time serial data plotter with accurate time scaling.
    
    Timing Details:
    - Microcontroller sends data at 10Hz (every 100ms)
    - Each sample represents 0.1 seconds of real time
    - X-axis shows actual time in seconds, not sample indices
    - Y-axis auto-scales based on data range
    """
    def __init__(self, serial_connection, max_points=100, graph_labels=None):
        self.ser = serial_connection
        self.max_points = max_points
        
        # Will be set dynamically
        self.graph_labels = graph_labels  # If provided, but we'll override
        self.data_streams = []
        self.timestamps = deque(maxlen=self.max_points)  # Store timestamps for proper time scaling
        self.start_time = None  # Record when we start receiving data
        self.fig = None
        self.axes = None
        self.lines = None
        
    def _update(self, frame):
        if self.fig is None:
            # First call: wait for data and setup figure
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    values = line.split(',')
                    n = len(values)
                    
                    # Create data streams
                    self.data_streams = [deque(maxlen=self.max_points) for _ in range(n)]
                    self.start_time = time.time()  # Record start time
                    # Set proper labels based on expected motor control data
                    if n == 11:
                        self.graph_labels = ['Position (rad)', 'Setpoint (rad)', 'Velocity (rad/s)', 
                                          'Acceleration (rad/s²)', 'Jerk (rad/s³)', 'Current (A)',
                                          'Motor Speed (%)', 'P Error', 'I Error', 'D Error', 'Reward']
                    else:
                        self.graph_labels = [f'Value {i+1}' for i in range(n)]
                    
                    # Create figure
                    if n <= 4:
                        self.fig, self.axes = plt.subplots(1, n, figsize=(4*n, 4))
                        if n == 1:
                            self.axes = [self.axes]
                    else:
                        rows = (n + 3) // 4
                        cols = min(n, 4)
                        self.fig, self.axes = plt.subplots(rows, cols, figsize=(4*cols, 4*rows))
                        self.axes = self.axes.flatten()
                    
                    self.lines = []
                    for i, ax in enumerate(self.axes[:n]):
                        line, = ax.plot([], [], 'b-', linewidth=2)
                        self.lines.append(line)
                        ax.set_ylim(-10, 10)
                        ax.set_xlim(0, self.max_points * 0.1)  # 0.1s per sample at 10Hz
                        ax.set_title(self.graph_labels[i], fontsize=10)
                        ax.set_xlabel('Time (s)')
                        ax.grid(True, alpha=0.3)
                    
                    # Hide extra axes
                    for ax in self.axes[n:]:
                        ax.set_visible(False)
                    
                    plt.tight_layout()
                    
                    # Append the first data with timestamp
                    current_time = 0.0  # Start at 0
                    self.timestamps.append(current_time)
                    for i, value in enumerate(values):
                        self.data_streams[i].append(float(value))
                    
                    # Update plots
                    for i, (line, data) in enumerate(zip(self.lines, self.data_streams)):
                        if len(data) > 0 and len(self.timestamps) > 0:
                            line.set_data(list(self.timestamps), list(data))
                            y_min, y_max = min(data), max(data)
                            padding = max(1, (y_max - y_min) * 0.1) if y_max != y_min else 1
                            self.axes[i].set_ylim(y_min - padding, y_max + padding)
                    
                    return self.lines
            except (ValueError, UnicodeDecodeError):
                pass
            return []
        
        # Normal update
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                values = line.split(',')
                if len(values) == len(self.lines):
                    # Add timestamp (each sample is 0.1s apart at 10Hz)
                    if self.timestamps:
                        current_time = self.timestamps[-1] + 0.1
                    else:
                        current_time = 0.0
                    self.timestamps.append(current_time)
                    
                    for i, value in enumerate(values):
                        self.data_streams[i].append(float(value))
        except (ValueError, UnicodeDecodeError):
            pass
        
        # Update plots with proper time scaling
        for i, (line, data) in enumerate(zip(self.lines, self.data_streams)):
            if len(data) > 0 and len(self.timestamps) > 0:
                # Use timestamps for x-axis, data for y-axis
                time_data = list(self.timestamps)
                value_data = list(data)
                line.set_data(time_data, value_data)
                
                # Update y-axis limits
                y_min, y_max = min(value_data), max(value_data)
                padding = max(1, (y_max - y_min) * 0.1) if y_max != y_min else 1
                self.axes[i].set_ylim(y_min - padding, y_max + padding)
                
                # Update x-axis limits to show the time range
                if len(time_data) > 1:
                    time_min, time_max = min(time_data), max(time_data)
                    self.axes[i].set_xlim(time_min, time_max)
        
        return self.lines
    
    def start(self, interval=50):
        """
        Start the real-time animation
        
        Args:
            interval: Update interval in milliseconds (default: 50ms = 20 fps)
        """
        print("Waiting for serial data to determine number of graphs...")
        while self.fig is None:
            self._update(None)
            time.sleep(0.1)
        
        n = len(self.lines)
        print(f"Detected {n} values, creating {n} graphs...")
        
        self.ani = FuncAnimation(self.fig, self._update, interval=interval)
        plt.show()
    
    def close(self):
        """Close the serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()