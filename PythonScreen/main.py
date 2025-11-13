import serial
import serial.tools.list_ports
from serial_plotter import SerialPlotter

def find_serial_port(baud_rate=9600):
    # Get all available ports
    available_ports = serial.tools.list_ports.comports()
    
    # Try ACM ports first, then COM ports
    for port in available_ports:
        port_name = port.device
        if 'ACM' in port_name or 'COM' in port_name:
            try:
                test_ser = serial.Serial(port_name, baud_rate, timeout=1)
                print(f"Connected to {port_name}")
                return test_ser
            except serial.SerialException:
                continue
    
    return None


def main():
    BAUD_RATE = 115200
    MAX_POINTS = 100
    ser = find_serial_port(BAUD_RATE) # Find and connect to serial port
    
    if ser is None:
        raise Exception("No valid serial port found. Please check your connections.")
    
    # Custom graph labels
    graph_labels = [
        'Angular Position (rad)', 
        'Angular Velocity (rad/s)', 
        'Angular Acceleration (rad/s²)', 
        'Angular Jerk (rad/s³)',
        'Current (A)',
        'Control Signal',
        'Reward'
    ]
    
    # Create and start the plotter
    plotter = SerialPlotter(ser, max_points=MAX_POINTS, graph_labels=graph_labels)
    
    print("Starting plotter... If no window appears, check your display settings.")
    print("Data format expected: pos,vel,acc,jrk,current,control_signal,reward")
    
    try:
        plotter.start(interval=100)  # Slower update rate
        print("Press Ctrl+C to stop the plotter.")
    finally:
        # Print maximum values recorded
        if hasattr(plotter, 'data_streams') and len(plotter.data_streams) >= 7:
            try:
                max_velocity = max(plotter.data_streams[1]) if plotter.data_streams[1] else 0
                max_acceleration = max(plotter.data_streams[2]) if plotter.data_streams[2] else 0
                max_jerk = max(plotter.data_streams[3]) if plotter.data_streams[3] else 0
                
                print("\nMaximum values recorded:")
                print(f"Max Velocity: {max_velocity:.3f} rad/s")
                print(f"Max Acceleration: {max_acceleration:.3f} rad/s²")
                print(f"Max Jerk: {max_jerk:.3f} rad/s³")
            except (ValueError, TypeError):
                print("Could not calculate maximum values - no valid data received.")
        
        plotter.close()


if __name__ == "__main__":
    main()