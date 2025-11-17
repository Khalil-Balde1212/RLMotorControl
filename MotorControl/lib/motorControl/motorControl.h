#include <Arduino.h>
#include "motorState.h"

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// Encoder pins
#define ENCODER_A 2
#define ENCODER_B 3
#define MOTOR_PIN_A 5
#define MOTOR_PIN_B 6
#define LEFT_LIMIT_SWITCH_PIN 0
#define RIGHT_LIMIT_SWITCH_PIN 8
#define CURRENT_SENSE_PIN A0


// Task frequencies (Hz)
#define SENSOR_TASK_FREQUENCY_HZ 50
#define MOTOR_CONTROL_TASK_FREQUENCY_HZ 1000

namespace Motor
{
    // Control output
    extern int motorSpeed; // Motor speed command (-255 to 255)

    // Setup and task functions
    void setup();
    void TaskSensorReads();
    void TaskMotorControl();
}

// Provide access to motor state variables through MotorState namespace
// This allows backward compatibility while using the new architecture
namespace Motor
{
    // Inline accessors for commonly used state variables
    inline float getPosition() { return MotorState::motorPos; }
    inline float getVelocity() { return MotorState::motorVel; }
    inline float getAcceleration() { return MotorState::motorAcc; }
    inline float getJerk() { return MotorState::motorJrk; }
    inline float getCurrent() { return MotorState::motorCurrent; }
    inline float getSetpoint() { return MotorState::motorSetpoint; }
    inline float getPropError() { return MotorState::propError; }
    inline float getIntError() { return MotorState::intError; }
    inline float getDerivError() { return MotorState::derivError; }
}

#endif