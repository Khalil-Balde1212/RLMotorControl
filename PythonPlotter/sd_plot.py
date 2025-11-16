import numpy as np
import matplotlib.pyplot as plt
import time


class SDPlot:
    def __init__(self, sd_decimate=10, max_history=100, display_interval: float | None = None, scroll_size: int = 20, include_si: bool = False, si_update_every: int = 10, si_display_interval: float | None = None):
        self.sd_decimate = max(1, sd_decimate)
        self.max_history = max_history
        self.display_interval = display_interval
        self.last_display_time = 0.0
        self._last_index = 0

        # create a combined figure; optionally include SI plots in the same
        # window. We'll allocate rows for SD and SI and construct axes
        # accordingly.
        if include_si:
            # 3 rows x 4 columns layout — left two columns for SD (6 plots),
            # right two columns for SI (6 plots) so they appear side-by-side.
            self.fig = plt.figure(figsize=(16, 9))
            gs = self.fig.add_gridspec(3, 4)
            # left half SD axes
            self.sd_axes = [self.fig.add_subplot(gs[i, j]) for i in range(3) for j in range(2)]
            # right half SI axes
            self.si_axes = [self.fig.add_subplot(gs[i, j]) for i in range(3) for j in range(2, 4)]
            self.fig.suptitle('SD (left) + SI (right)')
        else:
            # single-window without SI: use 3x2 SD grid
            self.fig = plt.figure(figsize=(10, 8))
            gs = self.fig.add_gridspec(3, 2)
            self.sd_axes = [self.fig.add_subplot(gs[i, j]) for i in range(3) for j in range(2)]
            self.si_axes = None

        # unified labels: pos, setpoint, vel, acc, jerk, current, motor_command
        self.sd_labels = ['pos', 'setpoint', 'vel', 'acc', 'jerk', 'current', 'motor_command']
        self.sd_raw = []
        self.sd_histories = {k: [] for k in self.sd_labels}

        # plot lines for each sensor (setpoint is overlay on pos)
        self.sd_lines = {}
        self.sd_axis_map = {}
        for ax, lbl in zip(self.sd_axes, [l for l in self.sd_labels if l != 'setpoint']):
            ln, = ax.plot([], [], label=lbl)
            ax.set_title(lbl)
            ax.set_xlabel('Decimated sample index')
            ax.grid(True)
            self.sd_lines[lbl] = ln
            self.sd_axis_map[lbl] = ax

        # setpoint overlay in pos axis
        self.pos_ax = self.sd_axes[0]
        self.setpoint_line, = self.pos_ax.plot([], [], 'k--', linewidth=1, label='setpoint')
        self.pos_ax.legend(loc='upper right')

        self.sd_count = 0
        # blit/live update support
        self.blit_enabled = False
        self._blit_bg = None
        self._blit_frame = 0
        # text display is disabled; prefer visual-only updates for SD

        # autoscroll: how many samples to show at once (0 to disable)
        self.scroll_size = max(0, int(scroll_size) if scroll_size is not None else 0)

        # SI-specific state when included
        self.include_si = include_si
        if include_si:
            # allocate SI plotting state similar to SIPlot
            self.si_max_history = max_history
            self.si_update_every = si_update_every
            self.si_display_interval = si_display_interval
            self.si_last_draw_time = 0.0
            self.si_update_counter = 0
            self.si_state_transition_matrix = np.eye(4)
            self.si_prev_state_transition_matrix = np.eye(4)
            self.si_response_vector = np.zeros(4)
            self.si_prev_response_vector = np.zeros(4)
            self.si_conv_history = []
            self.si_error_history = []
            self.si_matrix_history = []
            self.si_response_history = []
            # create images and bars on SI axes
            self.si_im = self.si_axes[0].imshow(self.si_state_transition_matrix, cmap='viridis', aspect='auto', vmin=-1, vmax=1)
            self.si_axes[0].set_title('SI: Current State Transition Matrix')
            plt.colorbar(self.si_im, ax=self.si_axes[0])
            self.si_im_change = self.si_axes[1].imshow(np.zeros((4, 4)), cmap='RdYlBu', aspect='auto', vmin=-0.01, vmax=0.01)
            self.si_axes[1].set_title('SI: State Transition Matrix Change')
            plt.colorbar(self.si_im_change, ax=self.si_axes[1])
            self.si_bars = self.si_axes[2].bar(range(len(self.si_response_vector)), self.si_response_vector, color='skyblue')
            self.si_axes[2].set_title('SI: Current Response Vector')
            self.si_im_vector = self.si_axes[3].imshow(np.zeros((1, 4)), cmap='RdYlBu', aspect='auto', vmin=-0.001, vmax=0.001)
            self.si_axes[3].set_title('SI: Response Vector Change')
            self.si_axes[4].set_title('SI: Current Error')
            self.si_axes[5].set_title('SI: Error Rate (Convergence)')
    
    def update_si(self, si_data):
        """Update SI plots that live inside the SD figure."""
        if not self.include_si or si_data is None:
            return

        self.si_state_transition_matrix = si_data.matrix
        self.si_response_vector = si_data.vector

        change_matrix = self.si_state_transition_matrix - self.si_prev_state_transition_matrix
        self.si_prev_state_transition_matrix = self.si_state_transition_matrix.copy()
        change_vector = self.si_response_vector - self.si_prev_response_vector
        self.si_prev_response_vector = self.si_response_vector.copy()

        self.si_conv_history.append(si_data.conv_rate)
        self.si_error_history.append(si_data.curr_error)
        self.si_matrix_history.append(self.si_state_transition_matrix.copy())
        self.si_response_history.append(self.si_response_vector.copy())

        if len(self.si_conv_history) > self.si_max_history:
            self.si_conv_history.pop(0); self.si_error_history.pop(0)
            self.si_matrix_history.pop(0); self.si_response_history.pop(0)

        self.si_update_counter += 1

        # determine if we should draw now
        now = time.time()
        should_draw = False
        if self.si_display_interval is not None:
            if now - self.si_last_draw_time >= self.si_display_interval:
                should_draw = True
                self.si_last_draw_time = now
        else:
            if self.si_update_counter % self.si_update_every == 0:
                should_draw = True

        if not should_draw:
            return

        try:
            self.si_im.set_data(self.si_state_transition_matrix)
            self.si_im_change.set_data(change_matrix)
            for bar, val in zip(self.si_bars, self.si_response_vector):
                bar.set_height(val)
            try:
                ymin = min(self.si_response_vector) - 1e-4
                ymax = max(self.si_response_vector) + 1e-4
            except Exception:
                ymin, ymax = -1e-3, 1e-3
            self.si_axes[2].set_ylim(ymin, ymax)
            self.si_im_vector.set_data(change_vector.reshape(1, 4))
            self.si_axes[4].clear(); self.si_axes[4].plot(range(len(self.si_error_history)), self.si_error_history, 'r-')
            self.si_axes[5].clear(); self.si_axes[5].plot(range(len(self.si_conv_history)), self.si_conv_history, 'b-')
            plt.draw(); plt.pause(0.001)
        except Exception:
            pass
    def update(self, sd_data, live=False):
        if sd_data is None:
            return

        # Removed text updates from SD to keep the console clean and reduce
        # overhead during live updates. The SD graph will update visually but
        # we won't display the pretty text block any more.

        # append raw
        setpoint = sd_data.setpoint if sd_data.setpoint is not None else float('nan')
        self.sd_raw.append((sd_data.pos, setpoint, sd_data.vel, sd_data.acc, sd_data.jerk, sd_data.current, sd_data.motor_command))
        self.sd_count += 1

        updated_hist = False
        if live and self.display_interval is None:
            # immediate: add last raw as decimated point
            latest = self.sd_raw[-1]
            mean_vals = [float(np.nanmean([latest[i]])) for i in range(7)]
            updated_hist = True
        else:
            # If a time-based display interval is configured, we don't rely on
            # a fixed sample window. Instead collect all new samples since the
            # last display interval and average them.
            if self.display_interval is not None:
                now = __import__('time').time()
                if now - self.last_display_time >= self.display_interval and self._last_index < len(self.sd_raw):
                    window = self.sd_raw[self._last_index:]
                    mean_vals = [float(np.nanmean([s[i] for s in window])) for i in range(7)]
                    updated_hist = True
                    self._last_index = len(self.sd_raw)
                    self.last_display_time = now
            else:
                if self.sd_count % self.sd_decimate == 0:
                    window = self.sd_raw[-self.sd_decimate:]
                    mean_vals = [float(np.nanmean([s[i] for s in window])) for i in range(7)]
                    updated_hist = True
            

        if updated_hist:
            for k, v in zip(self.sd_labels, mean_vals):
                self.sd_histories[k].append(v)
                if len(self.sd_histories[k]) > self.max_history:
                    self.sd_histories[k].pop(0)

            # update lines for each sensor (pos setpoint handled separately)
            for k, ln in self.sd_lines.items():
                hist = self.sd_histories[k]
                if self.scroll_size > 0:
                    start = max(0, len(hist) - self.scroll_size)
                    xs = list(range(start, len(hist)))
                    ln.set_data(xs, hist[start:])
                    # keep the X window locked to the last N samples
                    ax = self.sd_axis_map.get(k)
                    if ax is not None:
                        ax.set_xlim(start, start + max(1, self.scroll_size if len(hist) >= self.scroll_size else len(hist)))
                else:
                    ln.set_data(range(len(hist)), hist)
                ax = self.sd_axis_map.get(k)
                if ax is not None:
                    ax.relim(); ax.autoscale_view()

            # overlay setpoint
            sp_vals = self.sd_histories['setpoint']
            if len(sp_vals) > 0 and not all(np.isnan(sp_vals)):
                if self.scroll_size > 0:
                    start = max(0, len(sp_vals) - self.scroll_size)
                    self.setpoint_line.set_data(range(start, len(sp_vals)), sp_vals[start:])
                else:
                    self.setpoint_line.set_data(range(len(sp_vals)), sp_vals)
            else:
                self.setpoint_line.set_data([], [])
            self.pos_ax.relim(); self.pos_ax.autoscale_view()

            # If blit is enabled, draw only artists and blit to the canvas for a
            # much smoother update. This avoids a full redraw which is slow.
            if live and self.blit_enabled:
                try:
                    canvas = self.fig.canvas
                    # restore background
                    canvas.restore_region(self._blit_bg)
                    # draw sensor lines to renderer region
                    for k, ln in self.sd_lines.items():
                        ax = self.sd_axis_map[k]
                        ax.draw_artist(ln)
                    # draw setpoint
                    self.pos_ax.draw_artist(self.setpoint_line)
                    # draw SI artists when included
                    if self.include_si:
                        try:
                            self.si_axes[0].draw_artist(self.si_im)
                            self.si_axes[1].draw_artist(self.si_im_change)
                            for bar in self.si_bars:
                                self.si_axes[2].draw_artist(bar)
                            self.si_axes[3].draw_artist(self.si_im_vector)
                        except Exception:
                            pass
                    
                    canvas.blit(self.fig.bbox)

                    # occasionally allow autoscale/redraw to update background
                    self._blit_frame += 1
                    if self._blit_frame % 20 == 0:
                        canvas.draw()
                        self._blit_bg = canvas.copy_from_bbox(self.fig.bbox)
                except Exception:
                    # fallback to normal draw
                    self.fig.canvas.draw_idle(); plt.pause(0.001)
            else:
                self.fig.canvas.draw_idle(); plt.pause(0.001)

    def enable_live(self):
        """Enable live blit-style updates for smoother real-time plotting.

        This does an initial draw to compute the background for blitting.
        """
        try:
            self.fig.canvas.draw()
            self._blit_bg = self.fig.canvas.copy_from_bbox(self.fig.bbox)
            self.blit_enabled = True
            self._blit_frame = 0
        except Exception:
            self.blit_enabled = False
