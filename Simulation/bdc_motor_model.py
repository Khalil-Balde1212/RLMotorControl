import numpy as np
from scipy import signal

# Motor Params - Estimated for DC 12V 130RPM Geared Motor
# Based on: https://www.amazon.ca/dp/B07X1STDH2
# Typical small DC motor with ~30:1 gearbox

# Base motor (before gearbox) estimated parameters:
R = 0.5       # Resistance (Ohms) - typical for small DC motors
L = 0.1     # Inductance (H) - small motors have low inductance
Ke = 0.0073  # Back EMF constant of base motor (V/rad/s)
Kt = 0.0073  # Torque constant of base motor (Nm/A)
J = 0.000001  # Base motor rotor inertia (kg.m^2)
B = 0.00001  # Base motor friction (minimal)

J += 0.0002  # Add gearbox inertia
B += 0.0001   # Add gearbox friction

J += 0.0005  # Add load inertia
B += 0.0005  # Add load friction

MAX_VOLTAGE = 12.0

# With 130 RPM at 12V:
# 130 RPM = 13.6 rad/s
# This validates our parameters: ω = V/(Ke + R*B/Kt) ≈ 13.6 rad/s ✓

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


# Calculate maximum motor performance from motor parameters
# Max steady-state velocity occurs when Back-EMF equals applied voltage at max torque
# At steady state: V_max = Ke * ω_max + R * I_ss
# Where I_ss is the current needed to overcome friction: B * ω_max = Kt * I_ss
# Solving: ω_max = V_max / (Ke + R * B / Kt)
MaxVel = MAX_VOLTAGE / (Ke + R * B / Kt)
MaxVel *= 1.2  # Safety margin

# Maximum acceleration occurs at stall (ω = 0) with maximum current
# I_max = V_max / R (at stall, no back-EMF)
# Max torque: T_max = Kt * I_max - B * 0 = Kt * V_max / R
# Max acceleration: α_max = T_max / J
MaxAcc = (Kt * MAX_VOLTAGE / R) / J
MaxAcc *= 1.2  # Safety margin

# Maximum jerk is limited by electrical time constant L/R
# di/dt_max = V_max / L (when current is zero)
# dα/dt_max = (Kt / J) * di/dt_max
MaxJrk = (Kt / J) * (MAX_VOLTAGE / L)
MaxJrk *= 1.2  # Safety margin




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
