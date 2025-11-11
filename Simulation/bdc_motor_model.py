import numpy as np
from scipy import signal

# Motor Params
R = 0.5       # Resistance (Ohms)
L = 0.01      # Inductance (H)
Ke = 0.01     # Back EMF constant (V/rad/s)
Kt = 0.01     # Torque constant (Nm/A)
J = 0.01      # Rotor inertia (kg.m^2)
B = 0.1       # Viscous friction coeff (Nm.s/rad)
MAX_VOLTAGE = 12.0

# State variables
angularPos = 0.0
angularVel = 0.0
angularAcc = 0.0
angularJrk = 0.0
motor_current = 0.0  # Current through motor
prev_angularVel = 0.0
prev_angularAcc = 0.0
lastControlEffort = 0.0
current_time = 0.0

# History for tracking
theta_history = []
thetadot_history = []
thetadotdot_history = []
thetadotdotdot_history = []

# Control Frequency (Hz)
CONTROL_FREQUENCY = 1000
DT = 1.0 / CONTROL_FREQUENCY # Discrete Timesteps




def motorStep(control_effort):
    global angularPos, angularVel, angularAcc, angularJrk, motor_current
    global lastControlEffort, current_time, prev_angularVel, prev_angularAcc
    
    # Convert control effort to voltage
    inputVoltage = MAX_VOLTAGE * np.clip(control_effort, -1.0, 1.0)
    
    # Motor electrical equation: V = R*i + L*di/dt + Ke*ω
    # Solve for di/dt: di/dt = (V - R*i - Ke*ω) / L
    di_dt = (inputVoltage - R * motor_current - Ke * angularVel) / L
    motor_current += di_dt * DT
    
    # Motor mechanical equation: J*dω/dt = Kt*i - B*ω
    # Solve for dω/dt (angular acceleration)
    angularAcc = (Kt * motor_current - B * angularVel) / J
    
    # Update angular velocity
    angularVel += angularAcc * DT
    
    # Update angular position
    angularPos += angularVel * DT
    
    # Calculate jerk
    angularJrk = (angularAcc - prev_angularAcc) / DT
    
    # Store previous values
    prev_angularVel = angularVel
    prev_angularAcc = angularAcc
    
    # Update time
    current_time += DT
    
    # Update history
    theta_history.append(angularPos)
    thetadot_history.append(angularVel)
    thetadotdot_history.append(angularAcc)
    thetadotdotdot_history.append(angularJrk)
    
    # Update last control
    lastControlEffort = control_effort
    
    return angularPos, angularVel, angularAcc, angularJrk



def reset_motor_state():
    """Reset motor to initial state"""
    global angularPos, angularVel, angularAcc, angularJrk, motor_current
    global lastControlEffort, current_time, prev_angularVel, prev_angularAcc
    global theta_history, thetadot_history, thetadotdot_history, thetadotdotdot_history
    
    angularPos = 0.0
    angularVel = 0.0
    angularAcc = 0.0
    angularJrk = 0.0
    motor_current = 0.0
    prev_angularVel = 0.0
    prev_angularAcc = 0.0
    lastControlEffort = 0.0
    current_time = 0.0
    
    theta_history = []
    thetadot_history = []
    thetadotdot_history = []
    thetadotdotdot_history = []


def get_motor_state():
    """Return current motor state"""
    return {
        'position': angularPos,
        'velocity': angularVel,
        'acceleration': angularAcc,
        'jerk': angularJrk,
        'control_effort': lastControlEffort,
        'time': current_time
    }
