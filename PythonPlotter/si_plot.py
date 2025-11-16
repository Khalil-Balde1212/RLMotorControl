import numpy as np
import matplotlib.pyplot as plt


class SIPlot:
    def __init__(self, max_history=100, update_every=10, debug=False, display_interval: float | None = None):
        self.max_history = max_history
        self.update_every = update_every
        self.debug = debug
        self.display_interval = display_interval
        self.last_draw_time = 0.0
        self.pending_data = None

        self.state_transition_matrix = np.eye(4)
        self.prev_state_transition_matrix = np.eye(4)
        self.response_vector = np.zeros(4)
        self.prev_response_vector = np.zeros(4)

        self.conv_history = []
        self.error_history = []
        self.matrix_history = []
        self.response_history = []

        self.fig, self.axes = plt.subplots(3, 2, figsize=(12, 12))

        self.im = self.axes[0, 0].imshow(self.state_transition_matrix, cmap='viridis', aspect='auto', vmin=-1, vmax=1)
        self.axes[0, 0].set_title('Current State Transition Matrix')
        plt.colorbar(self.im, ax=self.axes[0, 0])

        self.im_change = self.axes[0, 1].imshow(np.zeros((4, 4)), cmap='RdYlBu', aspect='auto', vmin=-0.01, vmax=0.01)
        self.axes[0, 1].set_title('State Transition Matrix Change')
        plt.colorbar(self.im_change, ax=self.axes[0, 1])

        self.bars = self.axes[1, 0].bar(range(len(self.response_vector)), self.response_vector, color='skyblue')
        self.axes[1, 0].set_title('Current Response Vector')

        self.im_vector = self.axes[1, 1].imshow(np.zeros((1, 4)), cmap='RdYlBu', aspect='auto', vmin=-0.001, vmax=0.001)
        self.axes[1, 1].set_title('Response Vector Change')

        self.axes[2, 0].set_title('Current Error')
        self.axes[2, 1].set_title('Error Rate (Convergence)')

        self.update_counter = 0

    def update(self, si_data):
        if si_data is None:
            return

        self.state_transition_matrix = si_data.matrix
        self.response_vector = si_data.vector

        # compute changes
        change_matrix = self.state_transition_matrix - self.prev_state_transition_matrix
        self.prev_state_transition_matrix = self.state_transition_matrix.copy()
        change_vector = self.response_vector - self.prev_response_vector
        self.prev_response_vector = self.response_vector.copy()

        self.conv_history.append(si_data.conv_rate)
        self.error_history.append(si_data.curr_error)
        self.matrix_history.append(self.state_transition_matrix.copy())
        self.response_history.append(self.response_vector.copy())

        if len(self.conv_history) > self.max_history:
            self.conv_history.pop(0); self.error_history.pop(0)
            self.matrix_history.pop(0); self.response_history.pop(0)

        self.update_counter += 1
        self.pending_data = si_data
        if self.debug:
            print(f"SIPlot.update called, counter={self.update_counter}, update_every={self.update_every}, display_interval={self.display_interval}")

        # If a display interval is set, draw based on the elapsed time, otherwise
        # use the update_every counter as before.
        now = __import__('time').time()
        should_draw = False
        if self.display_interval is not None:
            if now - self.last_draw_time >= self.display_interval:
                should_draw = True
                self.last_draw_time = now
        else:
            # decimation based on update_every
            if self.update_counter % self.update_every == 0:
                should_draw = True

        if not should_draw:
            return

        try:
            self.im.set_data(self.state_transition_matrix)
            self.im_change.set_data(change_matrix)
            for bar, val in zip(self.bars, self.response_vector):
                bar.set_height(val)
            # avoid empty range errors on bar limits
            try:
                ymin = min(self.response_vector) - 1e-4
                ymax = max(self.response_vector) + 1e-4
            except Exception:
                ymin, ymax = -1e-3, 1e-3
            self.axes[1, 0].set_ylim(ymin, ymax)
            self.im_vector.set_data(change_vector.reshape(1, 4))
            self.axes[2, 0].clear(); self.axes[2, 0].plot(range(len(self.error_history)), self.error_history, 'r-')
            self.axes[2, 1].clear(); self.axes[2, 1].plot(range(len(self.conv_history)), self.conv_history, 'b-')
            plt.draw(); plt.pause(0.01)
            if self.debug:
                print("SIPlot: drew SI figure")
        except Exception:
            pass
