import numpy as np
import matplotlib.pyplot as plt
from bdc_motor_model import motorStep, reset_motor_state, get_motor_state, MaxVel, MaxAcc, MaxJrk
from typing import Optional

# Simple system identification simulation that mirrors MotorControl::SystemIdentification
# - A (stateTransition) initialized as identity
# - B (responseVector) initialized as ones
# - input: chirp + multi-sine similar to MCU
# - sampling frequency 50Hz with inner motorStep at 1000Hz

SAMPLING_HZ = 1000
INNER_STEPS = int(1000 / SAMPLING_HZ)  # motorStep runs at 1kHz
DT = 1.0 / SAMPLING_HZ

MAX_POSITION = 100.0

def normalize_state(pos, vel, acc, jerk, current):
    # Normalize position, velocity, acceleration, jerk, current
    MAX_CURRENT = 10.0
    return np.array([
        pos / MAX_POSITION,
        vel / MaxVel,
        acc / MaxAcc,
        jerk / MaxJrk,
        current / MAX_CURRENT
    ]).reshape(5, 1)


def run_sim(duration_sec=20.0, learning_rate=0.001, seed=0):
    # Excitation parameters (tuned for realistic motor operation)
    AMP1 = 0.8
    AMP2 = 0.5
    AMP3 = 0.3
    FREQ1_START, FREQ1_END = 0.01, 1   # Hz
    FREQ2_START, FREQ2_END = 0.5, 2   # Hz
    FREQ3_START, FREQ3_END = 1, 5.0   # Hz
    np.random.seed(seed)
    np.random.seed(seed)

    # Initialize model
    A = np.ones((5, 5))
    B = np.zeros((5, 1))  # Start with zeros for better convergence
    last_state = np.zeros((5, 1))

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

    regularization_lambda = 0.0001  # Regularization strength
    I = np.eye(5)

    batch_size = 5
    batch_errors = []
    batch_last_states = []
    batch_us = []
    batch_current_states = []
    for step in range(steps):
        time = step / SAMPLING_HZ
        freq = np.interp(min(time / 30.0, 1.0), [0.0, 1.0], [FREQ1_START, FREQ1_END])
        freq2 = np.interp(min(time / 25.0, 1.0), [0.0, 1.0], [FREQ2_START, FREQ2_END])
        freq3 = np.interp(min(time / 20.0, 1.0), [0.0, 1.0], [FREQ3_START, FREQ3_END])
        motor_sig = AMP1 * np.sin(2.0 * np.pi * freq * time)
        motor_sig += AMP2 * np.sin(2.0 * np.pi * freq2 * time)
        motor_sig += AMP3 * np.sin(2.0 * np.pi * freq3 * time)
        motor_sig += np.random.normal(0, 0.02)
        motor_sig = float(np.clip(motor_sig, -1.0, 1.0))

        for _ in range(INNER_STEPS):
            motorStep(motor_sig)

        ms = get_motor_state()
        pos, vel, acc, jerk, current = ms['position'], ms['velocity'], ms['acceleration'], ms['jerk'], ms['control_effort']
        current_state = normalize_state(pos, vel, acc, jerk, current)
        u = np.array([[motor_sig]])
        predicted = A @ last_state + B @ u
        error = current_state - predicted
        current_error_norm = np.linalg.norm(error)
        batch_errors.append(error)
        batch_last_states.append(last_state.copy())
        batch_us.append(u.copy())
        batch_current_states.append(current_state.copy())

        # Only update A, B every batch_size steps
        if (step + 1) % batch_size == 0:
            # Sum errors and updates over batch
            updateA = np.zeros_like(A)
            updateB = np.zeros_like(B)
            batch_update_norm = 0.0
            for i in range(batch_size):
                e = batch_errors[i]
                ls = batch_last_states[i]
                u_i = batch_us[i]
                updateA += learning_rate * (e @ ls.T)
                updateB += learning_rate * (e * u_i)
                batch_update_norm += np.linalg.norm(learning_rate * (e @ ls.T)) + np.linalg.norm(learning_rate * (e * u_i))
            # Average updates
            updateA /= batch_size
            updateB /= batch_size
            # Clip updates to prevent explosion
            updateA = np.clip(updateA, -1.0, 1.0)
            updateB = np.clip(updateB, -1.0, 1.0)
            # Regularize A towards identity
            A = A + updateA - regularization_lambda * (A - I)
            B = B + updateB
            # Clip matrices to prevent extreme values and zeros
            eps = 1e-6
            A = np.clip(A, -10.0, 10.0)
            A = np.where(np.abs(A) < eps, np.sign(A) * eps, A)
            B = np.clip(B, -10.0, 10.0)
            B = np.where(np.abs(B) < eps, np.sign(B) * eps, B)
            update_norm = batch_update_norm / batch_size
            batch_errors = []
            batch_last_states = []
            batch_us = []
            batch_current_states = []
        else:
            update_norm = 0.0

        ema_error_norm = ema_alpha * current_error_norm + (1.0 - ema_alpha) * ema_error_norm
        times.append(time)
        error_norms.append(current_error_norm)
        update_norms.append(update_norm)
        A_history.append(A.copy())
        B_history.append(B.copy())
        last_state = current_state.copy()
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
    # Check for error stagnation: if error hasn't decreased by more than 1% over last 10 samples, consider stagnated
    window = 10
    if len(error_norms) >= window:
        recent = error_norms[-window:]
        if np.max(recent) - np.min(recent) < 0.01 * np.max(recent):
            # Stagnation detected, return current time
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
    duration = 100.0  # seconds
    lr = 0.0005
    times, error_norms, update_norms, A_final, B_final, A_hist, B_hist = run_sim(duration_sec=duration, learning_rate=lr, seed=42)

    # Export LTI model as numpy arrays
    np.save('A.npy', A_final)
    np.save('B.npy', B_final)
    print("LTI model exported as A.npy and B.npy")
    print("Final A:\n", A_final)
    print("Final B:\n", B_final)

    # Check eigenvalues of A.npy for stability
    print("\nChecking eigenvalues of A.npy for stability:")
    A_loaded = np.load('A.npy')
    eigvals = np.linalg.eigvals(A_loaded)
    print("Eigenvalues of A:", eigvals)
    sampling_rate = SAMPLING_HZ if 'SAMPLING_HZ' in globals() else 500
    stable = True
    for idx, eig in enumerate(eigvals):
        crit = eig * (1.0 / sampling_rate) + 1.0
        passed = np.abs(crit) < 1.0
        print(f"Eigenvalue {idx}: {eig:.4f}, stability criterion: {crit:.4f} -> {'PASS' if passed else 'FAIL'}")
        if not passed:
            stable = False
    if stable:
        print("Model is stable (all eigenvalues pass the criterion eigen*dt + 1 < 1)")
    else:
        print("Warning: Model is unstable (some eigenvalues fail the criterion eigen*dt + 1 < 1)")

    # Calculate expected discrete LTI from bdc_motor_model.py
    from bdc_motor_model import R, L, Ke, Kt, J, B, DT
    # State: [pos, vel, acc, jerk, current]
    # Linearized continuous-time state-space:
    A_true = np.zeros((5,5))
    B_true = np.zeros((5,1))
    # pos_dot = vel
    A_true[0,1] = 1.0
    # vel_dot = acc
    A_true[1,2] = 1.0
    # acc_dot = jerk
    A_true[2,3] = 1.0
    # jerk_dot = (Kt/J) * current_dot - (B/J) * acc
    # current_dot = (-R/L)*current + (-Ke/L)*vel + (Kt/L)*u
    # So jerk_dot = (Kt/J)*[(-R/L)*current + (-Ke/L)*vel + (Kt/L)*u] - (B/J)*acc
    A_true[3,1] = (Kt/J) * (-Ke/L)
    A_true[3,2] = -B/J
    A_true[3,3] = (Kt/J) * (-R/L)
    B_true[3,0] = (Kt/J) * (Kt/L)
    # current_dot = (-R/L)*current + (-Ke/L)*vel + (Kt/L)*u
    A_true[4,1] = -Ke/L
    A_true[4,3] = -R/L
    B_true[4,0] = Kt/L
    print("\nContinuous-time A_true:\n", A_true)
    print("Continuous-time B_true:\n", B_true)
    # Discretize using exact matrix exponential for better accuracy
    from scipy.linalg import expm
    A_aug = np.zeros((6,6))
    A_aug[:5, :5] = A_true
    A_aug[:5, 5] = B_true.flatten()
    A_aug_exp = expm(A_aug * DT)
    A_true_discrete = A_aug_exp[:5, :5]
    B_true_discrete = A_aug_exp[:5, 5:6]
    print("\nExpected discrete LTI A (from motor model):\n", A_true_discrete)
    print("Expected discrete LTI B (from motor model):\n", B_true_discrete)

    # Compare identified vs true
    A_error = A_final - A_true_discrete
    B_error = B_final - B_true_discrete
    print("\nA error norm:", np.linalg.norm(A_error))
    print("B error norm:", np.linalg.norm(B_error))

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
    threshold = 0.05  # same as accuracyThreshold in RLAgent
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
    predicted = estimate_convergence_time(times, error_norms, threshold=threshold)
    if predicted is None:
        print(f"Estimated convergence time: unknown (insufficient data or error not decaying)")
    else:
        remaining = predicted - times[-1]
        if remaining <= 0:
            print(f"Estimated convergence time: already at or below threshold (t = {predicted:.2f} s)")
        else:
            print(f"Estimated convergence time: {predicted:.2f} s (approximately {remaining:.2f} s remaining)")
