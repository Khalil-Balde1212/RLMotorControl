import numpy as np
import matplotlib.pyplot as plt
import serial
import time
import serial.tools.list_ports

# Serial port configuration
BAUD_RATE = 115200  # Match the Arduino's Serial.begin(115200)

def parse_data(line):
    try:
        values = line.strip().split(',')
        if len(values) != 23:
            return None, None, None, None, None
        
        # 16 values for 4x4 matrix
        matrix_flat = [float(v) for v in values[:16]]
        matrix = np.array(matrix_flat).reshape(4, 4)
        
        # 4 values for vector
        vector_flat = [float(v) for v in values[16:20]]
        vector = np.array(vector_flat)
        
        conv_rate = float(values[20])
        curr_error = float(values[21])
        converged = bool(int(values[22]))
        
        return matrix, vector, conv_rate, curr_error, converged
    except Exception as e:
        print(f"Parsing error: {e}, skipping line")
        return None, None, None, None, None

# Find available ports
ports = [p.device for p in serial.tools.list_ports.comports() if 'ACM' in p.device or 'COM' in p.device]
if not ports:
    print("No ACM or COM ports found. Please check your Arduino connection.")
    exit(1)

# Try to open each port
ser = None
for port in ports:
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for serial to initialize
        print(f"Successfully opened port {port}")
        break
    except serial.SerialException as e:
        print(f"Failed to open {port}: {e}")
        continue

if ser is None:
    print("Could not open any serial port.")
    exit(1)

# Initialize plot
plt.ion()
fig, axes = plt.subplots(3, 2, figsize=(12, 12))

# Initial data (dummy)
state_transition_matrix = np.eye(4)
prev_state_transition_matrix = np.eye(4)
response_vector = np.zeros(4)
prev_response_vector = np.zeros(4)
convergence_rate = 0.0
current_error = 0.0
converged = False

# Histories for live plotting
conv_history = []
error_history = []
max_history = 100  # Keep last 100 points

# Initial plots
im = axes[0,0].imshow(state_transition_matrix, cmap='viridis', aspect='auto', vmin=-1, vmax=1)
axes[0,0].set_title('Current State Transition Matrix')
axes[0,0].set_xlabel('Columns')
axes[0,0].set_ylabel('Rows')
cbar = plt.colorbar(im, ax=axes[0,0])

im_change = axes[0,1].imshow(np.zeros((4,4)), cmap='RdYlBu', aspect='auto', vmin=-0.01, vmax=0.01)
axes[0,1].set_title('State Transition Matrix Change')
axes[0,1].set_xlabel('Columns')
axes[0,1].set_ylabel('Rows')
cbar_change = plt.colorbar(im_change, ax=axes[0,1])

bars = axes[1,0].bar(range(len(response_vector)), response_vector, color='skyblue')
axes[1,0].set_title('Current Response Vector')
axes[1,0].set_xlabel('Index')
axes[1,0].set_ylabel('Value')

im_vector = axes[1,1].imshow(np.zeros((1,4)), cmap='RdYlBu', aspect='auto', vmin=-0.001, vmax=0.001)
axes[1,1].set_title('Response Vector Change')
axes[1,1].set_xlabel('Index')
axes[1,1].set_ylabel('')

# Initial error plot
axes[2,0].plot([], [], 'r-')
axes[2,0].set_title('Current Error')
axes[2,0].set_xlabel('Steps')
axes[2,0].set_ylabel('Error')

# Initial convergence plot
axes[2,1].plot([], [], 'b-')
axes[2,1].set_title('Error Rate (Convergence)')
axes[2,1].set_xlabel('Steps')
axes[2,1].set_ylabel('Rate')

plt.tight_layout()

try:
    update_counter = 0
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                matrix, vector, conv_rate, curr_error, conv = parse_data(line)
                if matrix is not None and vector is not None and conv_rate is not None and curr_error is not None and conv is not None:
                    state_transition_matrix = matrix
                    response_vector = vector
                    convergence_rate = conv_rate
                    current_error = curr_error
                    converged = conv
                    
                    # Compute changes
                    change_matrix = state_transition_matrix - prev_state_transition_matrix
                    prev_state_transition_matrix = state_transition_matrix.copy()
                    change = response_vector - prev_response_vector
                    prev_response_vector = response_vector.copy()
                    
                    # Update histories
                    conv_history.append(convergence_rate)
                    error_history.append(current_error)
                    
                    # Keep only last max_history points
                    if len(conv_history) > max_history:
                        conv_history.pop(0)
                        error_history.pop(0)
                    
                    update_counter += 1
                    if update_counter % 10 == 0:  # Update plots every 10 blocks to reduce load
                        try:
                            print("Updating plots...")
                            # Update current STM
                            im.set_data(state_transition_matrix)
                            
                            # Update STM change
                            im_change.set_data(change_matrix)
                            
                            # Update current RV
                            for bar, val in zip(bars, response_vector):
                                bar.set_height(val)
                            min_val = min(response_vector) - 0.0001
                            max_val = max(response_vector) + 0.0001
                            if max_val - min_val < 1e-3:
                                max_val = min_val + 1e-3
                            axes[1,0].set_ylim(min_val, max_val)
                            
                            # Update RV change
                            im_vector.set_data(change.reshape(1,4))
                            
                            # Update error plot
                            axes[2,0].clear()
                            axes[2,0].plot(range(len(error_history)), error_history, 'r-')
                            axes[2,0].set_title('Current Error')
                            axes[2,0].set_xlabel('Steps')
                            axes[2,0].set_ylabel('Error')
                            if error_history:
                                min_y = 0
                                max_y = max(error_history) * 1.1
                                if max_y < 1e-3:
                                    max_y = 1e-3
                                axes[2,0].set_ylim(min_y, max_y)
                            
                            # Update convergence plot
                            axes[2,1].clear()
                            axes[2,1].plot(range(len(conv_history)), conv_history, 'b-')
                            axes[2,1].set_title('Error Rate (Convergence)')
                            axes[2,1].set_xlabel('Steps')
                            axes[2,1].set_ylabel('Rate')
                            if conv_history:
                                min_y = 0
                                max_y = max(conv_history) * 1.1
                                if max_y < 1e-5:
                                    max_y = 1e-5
                                axes[2,1].set_ylim(min_y, max_y)
                            
                            plt.draw()
                            plt.pause(0.01)
                        except Exception as e:
                            print(f"Plot update error: {e}")
                            continue
                    
                    # Print scalars
                    print(f"Convergence Rate: {convergence_rate}")
                    print(f"Current Error: {current_error}")
                    print(f"Converged: {converged}")
        except OSError as e:
            print(f"Serial read error: {e}, retrying...")
            time.sleep(1)
            continue
        
        time.sleep(0.01)  # Small delay

except Exception as e:
    print(f"Unexpected error: {e}")
    ser.close()
    plt.close()