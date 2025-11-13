import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque


class SerialPlotter:    
    def __init__(self, serial_connection, max_points=100, graph_labels=None):
        self.ser = serial_connection
        self.max_points = max_points
        
        # Default labels if none provided
        if graph_labels is None:
            graph_labels = ['Angular Position (rad)', 'Angular Velocity (rad/s)', 
                          'Angular Acceleration (rad/s²)', 'Angular Jerk (rad/s³)',
                          'Current (A)', 'Control Signal', 'Reward']
        self.graph_labels = graph_labels
        
        # Data storage for 7 values
        self.data_streams = [
            deque(maxlen=max_points),
            deque(maxlen=max_points),
            deque(maxlen=max_points),
            deque(maxlen=max_points),
            deque(maxlen=max_points),
            deque(maxlen=max_points),
            deque(maxlen=max_points),
        ]
        
        # Setup the figure and plots
        self._setup_figure()
        
    def _setup_figure(self):
        """Create the matplotlib figure with graphs"""
        # Create figure with graphs in a 2x4 grid for 7 plots + 1 empty
        self.fig, self.axes = plt.subplots(2, 4, figsize=(16, 8))
        self.axes = self.axes.flatten()
        self.lines = []
        
        for i, ax in enumerate(self.axes[:7]):  # Only use first 7 subplots
            line, = ax.plot([], [], 'b-', linewidth=2)
            self.lines.append(line)
            ax.set_ylim(-10, 10)
            ax.set_xlim(0, self.max_points)
            ax.set_title(self.graph_labels[i], fontsize=10, fontweight='bold')
            ax.set_xlabel('Time (0.1s)')
            ax.set_ylabel('Value')
            ax.grid(True, alpha=0.3)
        
        # Hide the 8th subplot
        self.axes[7].set_visible(False)
        
        plt.tight_layout()
    
    def _update(self, frame):
        """Update function called by animation"""
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                values = line.split(',')
                if len(values) == 7:
                    # Append new data to each stream
                    for i, value in enumerate(values):
                        self.data_streams[i].append(float(value))
        except (ValueError, UnicodeDecodeError) as e:
            # Silently ignore parsing errors
            pass
        
        # Always update plots and scaling
        for i, (line, data) in enumerate(zip(self.lines, self.data_streams)):
            if len(data) > 0:
                line.set_data(range(len(data)), list(data))
                # Auto-scale y-axis with some padding
                y_min, y_max = min(data), max(data)
                padding = max(1, (y_max - y_min) * 0.1)
                self.axes[i].set_ylim(y_min - padding, y_max + padding)
        
        return self.lines
    
    def start(self, interval=50):
        """
        Start the real-time animation
        
        Args:
            interval: Update interval in milliseconds (default: 50ms = 20 fps)
        """
        self.ani = FuncAnimation(self.fig, self._update, interval=interval)
        plt.show()
    
    def close(self):
        """Close the serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()