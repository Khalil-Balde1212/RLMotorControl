import numpy as np

import matplotlib.pyplot as plt

import bdc_motor_model as motor_model

posData = []
velData = []
accData = []
jrkData = []

angularVel_values = []
for i in range(2000):
    angularPos, angularVel, angularAcc, angularJrk = motor_model.motorStep(np.random.uniform(-1.0, 1.0))
    posData.append(angularPos)
    velData.append(angularVel)
    accData.append(angularAcc)
    jrkData.append(angularJrk)


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