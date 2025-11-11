import numpy as np

import matplotlib.pyplot as plt

import bdc_motor_model as motor_model

posData = []
velData = []
accData = []
jrkData = []

angularVel_values = []
for i in range(100):
    angularPos, angularVel, angularAcc, angularJrk = motor_model.motorStep(0.5)
    posData.append(angularPos)
    velData.append(angularVel)
    accData.append(angularAcc)
    jrkData.append(angularJrk)


# Create the plot
plt.figure(figsize=(10, 6))
plt.plot(velData, linewidth=2)
plt.xlabel('Sample Index')
plt.ylabel('Motor Step Value')
plt.title('Motor Step Data')
plt.grid(True)
plt.show()