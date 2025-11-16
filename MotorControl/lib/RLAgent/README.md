RLPolicy
================
# Is a neural Net :o

## Inputs
normalized state [pos, vel, acc, jrk, current]
nromalized state t+1 [post t+1, vel t+1, acc t+1, jrk t+1m current t+1]
position setpoint

total input layer: 11 nodes

## Hidden Layers
Layer 1: some amount of ReLU nodes
Layer 2: Lol there is none

## Outputs 
5 motor command trajectory
^ Task runs at 500Hz while motor runs at 1000Hz, which should leave the motors with 2.5 time steps worth of commands to use

# Reward
The RL policy reward is computed on predicted position errors (not velocity). The network simulates HORIZON steps of the LTI model and the reward uses the predicted position vs the setpoint.

# Short API:
- initializePolicy()
- computeMotorSequence(stateNorm, prevU, setpoint)
- calculateReward()
- updatePolicyWeights()
 - getCachedPos() // returns last predicted positions for telemetry

Serial debug:
- Send "rl on" or "rl off" over serial to toggle the neural policy telemetry (main_debugTasks)
- Send "rl reward" to print the most recent reward calculation
