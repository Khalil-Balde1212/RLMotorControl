#include "motorControl.h"
#include <mbed.h>
#include <chrono>
#include "motorState.h"

// =============================================================================
// Constants and Configuration
// =============================================================================


// =============================================================================
// Motor Control Namespace
// =============================================================================

namespace Motor
{
    // Control output
    int motorSpeed = 0;           // Motor speed command (-255 to 255)
}

// =============================================================================
// Setup and Initialization
// =============================================================================

void Motor::setup()
{
    // Initialize motor state
    MotorState::initialize();

    // Initialize encoder pins with pull-up resistors
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);

    // Attach interrupts for quadrature encoder decoding
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), MotorState::encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), MotorState::encoderISR, CHANGE);

    // Initialize motor control pins
    pinMode(MOTOR_PIN_A, OUTPUT);
    pinMode(MOTOR_PIN_B, OUTPUT);

    // Initialize current sensor pin
    pinMode(CURRENT_SENSE_PIN, INPUT);

    // Configure ADC for 12-bit resolution
    analogReadResolution(12);

    // Sensor and motor tasks should be started from main to allow centralized control
}

// =============================================================================
// Task Functions
// =============================================================================

void Motor::TaskSensorReads()
{
    // Task timing configuration
    const int delayMs = 1000 / SENSOR_TASK_FREQUENCY_HZ;
    float dt = 1.0f / SENSOR_TASK_FREQUENCY_HZ; // Time step in seconds

    while (true) {
        // Update all motor states using the state management system
        MotorState::updateAllStates(dt);
        // Maintain task timing
    rtos::ThisThread::sleep_for(std::chrono::milliseconds(delayMs));
    }
}

void Motor::TaskMotorControl()
{
    const int delayMs = 1000 / MOTOR_CONTROL_TASK_FREQUENCY_HZ;

    while (true) {
        // Control motor based on speed command
        if (Motor::motorSpeed > 0) {
            // Forward rotation
            analogWrite(MOTOR_PIN_A, Motor::motorSpeed);
            analogWrite(MOTOR_PIN_B, 0);
        }
        else if (Motor::motorSpeed < 0) {
            // Reverse rotation
            analogWrite(MOTOR_PIN_A, 0);
            analogWrite(MOTOR_PIN_B, -Motor::motorSpeed);
        }
        else {
            // Stop motor
            analogWrite(MOTOR_PIN_A, 0);
            analogWrite(MOTOR_PIN_B, 0);
        }

    rtos::ThisThread::sleep_for(std::chrono::milliseconds(delayMs));
    }
}



