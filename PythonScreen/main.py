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
    
    # Custom graph labels (will be overridden dynamically)
    graph_labels = None
    
    # Create and start the plotter
    plotter = SerialPlotter(ser, max_points=MAX_POINTS, graph_labels=graph_labels)
    
    print("Starting plotter... If no window appears, check your display settings.")
    print("Data format: comma-separated values (number of graphs will auto-adjust)")
    
    try:
        plotter.start(interval=100)  # Slower update rate
        print("Press Ctrl+C to stop the plotter.")
    finally:
        # Print maximum values recorded
        if hasattr(plotter, 'data_streams') and plotter.data_streams:
            n = len(plotter.data_streams)
            print(f"\nMaximum values recorded for {n} streams:")
            for i in range(n):
                if plotter.data_streams[i]:
                    max_val = max(plotter.data_streams[i])
                    print(f"Stream {i+1}: {max_val:.3f}")
                else:
                    print(f"Stream {i+1}: No data")
        
        plotter.close()


if __name__ == "__main__":
    main()