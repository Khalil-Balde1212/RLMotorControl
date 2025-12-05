import numpy as np

import matplotlib.pyplot as plt

import bdc_motor_model as motor_model

posData = []
velData = []
accData = []
jrkData = []


setpoint = 3.14
lastError = 0.0
integralError = 0.0

# Ku = 0.05
# Pu = 35s

kp = 0.006
ki = 0.000 
kd = 0.0

angularVel_values = []
for i in range(2000):
    # Chirp: setpoint sweeps from 0 to 2*pi over time
    t = i * 0.01  # time in seconds (100Hz)
    # Chirp: setpoint sweeps between 0.5 Hz and 3 Hz over time
    f0 = 0.1  # Start frequency (Hz)
    f1 = 1.0  # End frequency (Hz)
    T = 20.0  # Duration of sweep (seconds)
    t = i * 0.01  # time in seconds (100Hz)
    # Linear frequency sweep (chirp)
    freq = f0 + (f1 - f0) * t / T
    setpoint = np.sin(2 * np.pi * freq * t)

    error = setpoint - motor_model.angularPos
    integralError += error * 0.01
    derivativeError = (error - lastError) / 0.01
    lastError = error

    control_effort = kp * error + ki * integralError + kd * derivativeError
    control_effort = setpoint
    angularPos, angularVel, angularAcc, angularJrk = motor_model.motorStep(control_effort)
    posData.append(angularPos)
    velData.append(angularVel)
    accData.append(angularAcc)
    jrkData.append(angularJrk)


    # Calculate the ultimate period of posData
    # Find zero crossings of the oscillation around the setpoint
    crossings = []
    for i in range(1, len(posData)):
        if (posData[i-1] - setpoint) * (posData[i] - setpoint) < 0:
            crossings.append(i)

    # Calculate period from consecutive crossings (2 crossings = half period)
    if len(crossings) >= 2:
        periods = []
        for i in range(1, len(crossings)):
            half_period = crossings[i] - crossings[i-1]
            periods.append(2 * half_period)
        
        ultimate_period_steps = np.mean(periods)
        ultimate_period_seconds = ultimate_period_steps * 0.01  # Convert to seconds (100Hz sampling)
        print(f"Ultimate Period: {ultimate_period_seconds:.4f} seconds ({ultimate_period_steps:.2f} steps)")
    else:
        print("Not enough oscillations detected to calculate period")
# Create the plot with subplots



fig, axs = plt.subplots(4, 1, figsize=(10, 12))

axs[0].plot(posData, linewidth=2)
axs[0].set_ylabel('Pos(rad)')
axs[0].set_title('MotorStep Position Response')
axs[0].grid(True)

axs[1].plot(velData, linewidth=2)
axs[1].set_ylabel('Vel(rad/s)')
axs[1].set_title('MotorStep Velocity Response')
axs[1].grid(True)

axs[2].plot(accData, linewidth=2)
axs[2].set_ylabel('Acc(rad/s²)')
axs[2].set_title('MotorStep Acceleration Response')
axs[2].grid(True)

axs[3].plot(jrkData, linewidth=2)
axs[3].set_ylabel('Jrk(rad/s³)')
axs[3].set_title('MotorStep Jerk Response')
axs[3].set_xlabel('Time Steps (100hz)')
axs[3].grid(True)

plt.tight_layout()
plt.show()