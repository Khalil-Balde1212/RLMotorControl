This folder contains a simple physics-based BDC motor model (`bdc_motor_model.py`) and a system identification simulator (`sim_system_identification.py`) that mimics the `RLAgent` updates from the MCU.

Usage:

- Run the simulator and plot results:

    python3 Simulation/sim_system_identification.py

- Adjust parameters at the top of the script: `SAMPLING_HZ`, `duration`, and the learning rate.

Notes:

- The simulator runs the motor model at 1 kHz while performing identification updates at 50 Hz (mirroring `MotorControl/src/main.cpp`).
- `A` is initialized as the identity matrix and `B` as ones; updates are the same outer-product rules used in `RLAgent::update`.
- The plotting shows error norm (how well the model predicts) and update norm (convergence rate), plus final A/B heatmaps.
