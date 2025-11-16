#include "trajectoryController.h"
#include <vector>
#include <mbed.h>
#include <chrono>
#include "PID.h"
#include "motorControl.h"
#include "motorState.h"
// main.h for sharing useTrajectory flag
#include "main.h"

using namespace std;

namespace TrajectoryController {
    static rtos::Thread controllerThread;
    static volatile bool running = false;
    static float Kp = 1.0f, Ki = 0.0f, Kd = 0.01f;

    // Linear interpolation of low-frequency velocities to high-frequency
    static vector<float> linear_interpolate(const vector<float>& v_low, float dt_low, float dt_high) {
        if (v_low.empty()) return {};
        int n_low = (int)v_low.size();
        // total time span = (n_low) * dt_low? The lowfreq velocities correspond to segments between positions
        // We'll assume v_low length N means N samples with dt_low spacing, produce sample counts
        int samples_per_segment = max(1, (int)round(dt_low / dt_high));
        vector<float> v_high;
        for (int i = 0; i < n_low - 1; ++i) {
            float a = v_low[i];
            float b = v_low[i+1];
            for (int s = 0; s < samples_per_segment; ++s) {
                float t = (float)s / (float)samples_per_segment;
                v_high.push_back(a + (b - a) * t);
            }
        }
        // push the final point
        v_high.push_back(v_low.back());
        return v_high;
    }

    static void controller_loop(vector<float> v_high, float dt_high) {
        float integral = 0.0f;
        float prevError = 0.0f;
        for (size_t t = 0; t < v_high.size() && running; ++t) {
            float v_meas = MotorState::motorVel; // measured velocity
            float error = v_high[t] - v_meas;
            int u = computePID(v_high[t], v_meas, integral, prevError, Kp, Ki, Kd, dt_high, 1000.0f);
            // computePID returns int command; apply saturation
            u = constrain(u, -255, 255);
            Motor::motorSpeed = u;
            rtos::ThisThread::sleep_for(std::chrono::milliseconds((int)round(dt_high * 1000.0f)));
        }
        running = false;
    }

    void startTrajectory(const vector<float>& x_target_lowfreq, float dt_low, float dt_high) {
        if (running) return;
        // compute lowfreq velocity targets from positions
        vector<float> v_low;
        for (size_t i = 0; i + 1 < x_target_lowfreq.size(); ++i) {
            v_low.push_back((x_target_lowfreq[i+1] - x_target_lowfreq[i]) / dt_low);
        }
        // interpolate
        vector<float> v_high = linear_interpolate(v_low, dt_low, dt_high);

        // Notify main that trajectory will take control
        useTrajectory = true;
        running = true;
        controllerThread.start([v_high, dt_high]() mutable { controller_loop(v_high, dt_high); });
    }

    void stopTrajectory() {
    running = false;
    // release main controller takeover
    useTrajectory = false;
    }

    bool isRunning() {
        return running;
    }

    void setGains(float kp, float ki, float kd) {
        Kp = kp; Ki = ki; Kd = kd;
    }
}
