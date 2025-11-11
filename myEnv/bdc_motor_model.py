import numpy as np
from sympy import symbols
from sympy.physics.control import TransferFunction
from scipy import signal

# Motor Params
R = 0.5       # Resistance (Ohms)
L = 0.01      # Inductance (H)
Kb = 0.01     # Back EMF constant (V/rad/s)
Kt = 0.01     # Torque constant (Nm/A)
J = 0.01      # Rotor inertia (kg.m^2)
B = 0.1       # Viscous friction coeff (Nm.s/rad)
MAX_VOLTAGE = 12.0

# State variables
angularPos = 0.0
angularVel = 0.0
angularAcc = 0.0
angularJrk = 0.0
lastControlEffort = 0.0
current_time = 0.0

# History for tracking
omega_history = []
omega_dot_history = []

# Control Frequency (Hz)
CONTROL_FREQUENCY = 1000
DT = 1.0 / CONTROL_FREQUENCY

# Transfer Functions: Omega(s)/V(s)
# Define Laplace variable
s = symbols('s')

TF_ThetaV = TransferFunction(Kt, J*L*s**3 + (J*R + B*L)*s**2 + (B*R + Kb*Kt)*s, s)

TF_OmegaV = TransferFunction(Kt, J*L*s**2 + (J*R + B*L)*s + (B*R + Kb*Kt), s)
TF_OmegaV_Num = np.array([float(Kt)])
TF_OmegaV_Den = np.array([float(J * L), float(J * R + B * L), float(B * R + Kb * Kt)])

# Create scipy transfer function for numerical computation
# scipy uses (num, den) format where both are coefficient arrays
scipy_sys = signal.TransferFunction(TF_OmegaV_Num, TF_OmegaV_Den)

# Pre-compute step response for the time range we'll use
t_max = 10.0  # Maximum simulation time
num_points = int(t_max / DT)
t_step_precompute, y_step_precompute = signal.step(scipy_sys, T=np.linspace(0, t_max, num_points))

# Create interpolation function for fast lookup
from scipy.interpolate import interp1d
omega_step_response_func = interp1d(t_step_precompute, y_step_precompute, 
                                     kind='cubic', bounds_error=False, fill_value=(0, y_step_precompute[-1]))




def motorStep(control_effort):
    global angularPos, angularVel, angularAcc, angularJrk, lastControlEffort, current_time
    
    # Convert control effort to voltage
    inputVoltage = MAX_VOLTAGE * np.clip(control_effort, -1.0, 1.0)
    
    # Update time
    current_time += DT
    
    # Calculate velocity using step response
    # For a step input V at t=0: ω(t) = V × step_response(t)
    try:
        angularVel = float(inputVoltage * omega_step_response_func(current_time))
    except:
        angularVel = 0.0
    
    
    # Update last control
    lastControlEffort = control_effort
    
    return angularPos, angularVel, angularAcc, angularJrk



def reset_motor_state():
    """Reset motor to initial state"""
    global angularPos, angularVel, angularAcc, angularJrk, lastControlEffort, current_time
    global omega_history, omega_dot_history
    
    angularPos = 0.0
    angularVel = 0.0
    angularAcc = 0.0
    angularJrk = 0.0
    lastControlEffort = 0.0
    current_time = 0.0
    
    omega_history = []
    omega_dot_history = []


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
