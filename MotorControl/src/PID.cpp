#include "PID.h"
#include <FreeRTOS.h>

int computePID(float setpoint, float measured, float& integral, float& previousError,
               float kp, float ki, float kd, float dt, float maxIntegral) {
    float error = setpoint - measured;
    integral += error * dt;
    if (integral > maxIntegral) integral = maxIntegral;
    else if (integral < -maxIntegral) integral = -maxIntegral;
    float derivative = (error - previousError) / dt;
    previousError = error;
    int output = (int)(kp * error + ki * integral + kd * derivative);
    return output;
}


