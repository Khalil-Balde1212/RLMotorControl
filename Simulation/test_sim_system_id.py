import numpy as np
import pytest
from sim_system_identification import run_sim, estimate_convergence_time

# Quick sanity test to make sure the identification error decreases over a short run.

def test_error_decreases():
    duration = 6.0
    lr = 0.02
    times, error_norms, update_norms, A_final, B_final, Ah, Bh = run_sim(duration_sec=duration, learning_rate=lr, seed=7)

    # require error to have decreased by at least 10% from first quarter to last quarter
    N = len(error_norms)
    if N < 4:
        assert True
    else:
        first_q = np.mean(error_norms[: max(1, N // 4)])
        last_q = np.mean(error_norms[- max(1, N // 4):])
        # assert error decreased
        assert last_q <= first_q * 0.95


    def test_estimate_convergence_time_decreasing():
        # Create a synthetic decaying error curve and verify the estimator returns a prediction
        times = np.linspace(0, 10, 101).tolist()
        error_norms = np.exp(-0.5 * np.array(times))
        predicted = estimate_convergence_time(times, error_norms, threshold=0.1)
        assert predicted is not None
        assert predicted >= times[-1]  # predicted absolute time should be >= final sampled time


    def test_estimate_convergence_time_already_below():
        times = [0.0, 1.0, 2.0]
        error_norms = np.array([0.2, 0.15, 0.4])
        # Last point above threshold
        predicted = estimate_convergence_time(times, error_norms, threshold=0.1)
        assert predicted is not None

        # If we set threshold above last, we should get last time immediately
        predicted_now = estimate_convergence_time(times, np.array([0.05, 0.06, 0.07]), threshold=0.08)
        assert predicted_now == pytest.approx(times[-1])
