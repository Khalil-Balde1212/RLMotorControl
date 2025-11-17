import numpy as np
import matplotlib.pyplot as plt
from bdc_motor_model import motorStep, reset_motor_state, get_motor_state, MaxVel, MaxAcc, MaxJrk
from typing import Optional

# Simple system identification simulation that mirrors MotorControl::SystemIdentification
# - A (stateTransition) initialized as identity
# - B (responseVector) initialized as ones
# - input: chirp + multi-sine similar to MCU
# - sampling frequency 50Hz with inner motorStep at 1000Hz

SAMPLING_HZ = 50
INNER_STEPS = int(1000 / SAMPLING_HZ)  # motorStep runs at 1kHz
DT = 1.0 / SAMPLING_HZ

MAX_POSITION = 100.0

def normalize_state(pos, vel, acc, jrk):
    return np.array([pos / MAX_POSITION,
                     vel / MaxVel,
                     acc / MaxAcc,
                     jrk / MaxJrk]).reshape(4, 1)


def run_sim(duration_sec=20.0, learning_rate=0.01, seed=0):
    np.random.seed(seed)

    # Initialize model
    A = np.eye(4)
    B = np.ones((4, 1))

    last_state = np.zeros((4, 1))

    # Data history
    times = []
    error_norms = []
    update_norms = []
    A_history = []  # store some entries
    B_history = []

    # Reset motor
    reset_motor_state()

    t = 0.0
    steps = int(duration_sec * SAMPLING_HZ)
    ema_error_norm = 0.0
    ema_alpha = 0.1
    prev_error_norm = 0.0

    for step in range(steps):
        # Generate multi-chirp input similar to MCU
        time = step / SAMPLING_HZ
        # amplitude random per sample
        amp = np.random.uniform(0.5, 1.0)
        freq = np.interp(min(time / 30.0, 1.0), [0.0, 1.0], [0.2, 1.5])
        amp2 = np.random.uniform(0.3, 0.8)
        freq2 = np.interp(min(time / 25.0, 1.0), [0.0, 1.0], [0.33, 2.0])
        amp3 = np.random.uniform(0.2, 0.6)
        freq3 = np.interp(min(time / 20.0, 1.0), [0.0, 1.0], [0.5, 2.5])
        motor_sig = amp * np.sin(2.0 * np.pi * freq * time)
        motor_sig += (amp2 / 2.0) * np.sin(2.0 * np.pi * freq2 * time)
        motor_sig += (amp3 / 3.0) * np.sin(2.0 * np.pi * freq3 * time)
        motor_sig = float(np.clip(motor_sig, -1.0, 1.0))

        # Step the motor at 1kHz inner loop
        for _ in range(INNER_STEPS):
            motorStep(motor_sig)

        # Read motor state (in simulation motorStep updated global variables)
        ms = get_motor_state()
        pos, vel, acc, jrk = ms['position'], ms['velocity'], ms['acceleration'], ms['jerk']

        current_state = normalize_state(pos, vel, acc, jrk)

        # Input normalized (same as MCU uses motorScpeed / 255)
        u = np.array([[motor_sig]])

        # Predict next state using model A, B
        predicted = A @ last_state + B @ u

        # Error and update
        error = current_state - predicted
        current_error_norm = np.linalg.norm(error)

        updateA = learning_rate * (error @ last_state.T)
        updateB = learning_rate * (error * u)

        # Apply updates
        A = A + updateA
        B = B + updateB

        update_norm = np.linalg.norm(updateA) + np.linalg.norm(updateB)

        ema_error_norm = ema_alpha * current_error_norm + (1.0 - ema_alpha) * ema_error_norm

        times.append(time)
        error_norms.append(current_error_norm)
        update_norms.append(update_norm)
        A_history.append(A.copy())
        B_history.append(B.copy())

        last_state = current_state.copy()

        # Step time
        t += DT

    # Convert to arrays
    error_norms = np.array(error_norms)
    update_norms = np.array(update_norms)

    return times, error_norms, update_norms, A, B, A_history, B_history


def estimate_convergence_time(times: list, error_norms: np.ndarray, threshold: float = 0.5) -> Optional[float]:

    if len(times) == 0 or len(error_norms) == 0:
        return None

    # Already below threshold
    if float(error_norms[-1]) <= threshold:
        return float(times[-1])

    # Use only positive error values for log fit
    eps = 1e-12
    mask = error_norms > eps
    if mask.sum() < 3:
        # Not enough points to fit
        return None

    t_arr = np.array(times)[mask]
    e_arr = np.array(error_norms)[mask]

    # Fit log(e) = a + b * t -> e = exp(a) * exp(b * t)
    try:
        coeffs = np.polyfit(t_arr, np.log(e_arr), 1)
        b = coeffs[0]
        a = coeffs[1]
        # If slope is >= 0, error isn't decaying -> fallback
        if b >= 0:
            raise ValueError("Non-decaying error (b >= 0)")

        t_pred = (np.log(threshold) - a) / b
        if not np.isfinite(t_pred):
            return None
        # If prediction is earlier than current last sample, clamp to current time
        return float(max(t_pred, float(times[-1])))
    except Exception:
        # fallback: simple linear interpolation with last two points
        if len(times) >= 2 and error_norms[-2] > error_norms[-1]:
            t1, t2 = float(times[-2]), float(times[-1])
            e1, e2 = float(error_norms[-2]), float(error_norms[-1])
            # slope per second
            slope = (e2 - e1) / (t2 - t1) if (t2 - t1) != 0 else 0
            if slope >= 0:
                return None
            # Predict t when e = threshold: t = t2 + (threshold - e2) / slope
            t_pred = t2 + (threshold - e2) / slope
            return float(max(t_pred, t2))
        return None


if __name__ == '__main__':
    # Run simulation
    duration = 40.0  # seconds
    lr = 0.02
    times, error_norms, update_norms, A_final, B_final, A_hist, B_hist = run_sim(duration_sec=duration, learning_rate=lr, seed=42)

    # Plot results
    fig, axes = plt.subplots(2, 2, figsize=(10, 6))
    ax0 = axes[0, 0]
    ax1 = axes[0, 1]
    ax2 = axes[1, 0]
    ax3 = axes[1, 1]

    ax0.plot(times, error_norms, label='Error norm')
    ax0.set_title('Prediction Error Norm')

    ax1.plot(times, update_norms, label='Update norm', color='orange')
    ax1.set_title('Convergence (Update Norm)')

    # A_final heatmap
    im = ax2.imshow(A_final, cmap='RdBu', interpolation='nearest')
    ax2.set_title('Final A (State Transition)')
    fig.colorbar(im, ax=ax2)

    im2 = ax3.imshow(B_final, cmap='RdBu', interpolation='nearest')
    ax3.set_title('Final B (Response Vector)')
    fig.colorbar(im2, ax=ax3)

    # Estimate convergence time (absolute seconds) using the same threshold as MCU
    threshold = 0.5  # same as accuracyThreshold in RLAgent
    predicted = estimate_convergence_time(times, error_norms, threshold=threshold)
    if predicted is None:
        text_str = "Estimated convergence: unknown"
        print(f"Estimated convergence time: unknown (insufficient data or error not decaying)")
    else:
        remaining = predicted - times[-1]
        if remaining <= 0:
            text_str = f"Converged at {predicted:.1f}s"
            print(f"Estimated convergence time: already at or below threshold (t = {predicted:.2f} s)")
        else:
            text_str = f"Est. converged at {predicted:.1f}s ({remaining:.1f}s left)"
            print(f"Estimated convergence time: {predicted:.2f} s (approximately {remaining:.2f} s remaining)")

    # Overlay estimated convergence time on the error plot
    try:
        ax0.text(0.98, 0.95, text_str, transform=ax0.transAxes,
                 ha='right', va='top', fontsize=9,
                 bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    except Exception:
        pass

    plt.tight_layout()
    plt.show()

    # Estimate convergence time (absolute seconds) using the same threshold as MCU
    threshold = 0.5  # same as accuracyThreshold in RLAgent
    predicted = estimate_convergence_time(times, error_norms, threshold=threshold)
    if predicted is None:
        print(f"Estimated convergence time: unknown (insufficient data or error not decaying)")
    else:
        remaining = predicted - times[-1]
        if remaining <= 0:
            print(f"Estimated convergence time: already at or below threshold (t = {predicted:.2f} s)")
        else:
            print(f"Estimated convergence time: {predicted:.2f} s (approximately {remaining:.2f} s remaining)")
