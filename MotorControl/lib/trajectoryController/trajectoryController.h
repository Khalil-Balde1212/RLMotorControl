#ifndef TRAJECTORY_CONTROLLER_H
#define TRAJECTORY_CONTROLLER_H

#include <Arduino.h>
#include <vector>

namespace TrajectoryController {
	void startTrajectory(const std::vector<float>& x_target_lowfreq, float dt_low, float dt_high = 0.001f);
	void stopTrajectory();
	void setGains(float kp, float ki, float kd);
	bool isRunning();
}


#endif