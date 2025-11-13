#include <Arduino.h>
#ifndef PID_H
#define PID_H

int computePID(float setpoint, float measured, float &integral, float &previousError,
               float kp, float ki, float kd, float dt, float maxIntegral);
#endif