#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H

#include <Arduino.h>

// =============================================================================
// Constants and Configuration
// =============================================================================

// Encoder configuration
const int ENCODER_COUNTS_PER_REVOLUTION = 1940;

// Current sensor configuration (ACS712 5A)
// CALIBRATED VALUES for ACS712 5A sensor:
// - Standard sensitivity: 185mV/A = 0.185V/A
// - Your readings show ~0.12A when real current is 0.07A
// - Adjusted sensitivity: 0.185 * (0.12/0.07) = 0.316V/A
// - Center voltage adjusted so 1.27V gives 0.07A
// - Formula: current = (voltage - center) / sensitivity
const float CURRENT_SENSOR_VOLTAGE_CENTER = 1.248f;  // Adjusted center voltage
const float CURRENT_SENSOR_SENSITIVITY = 0.316f;    // Adjusted sensitivity (V/A)
const float ADC_REFERENCE_VOLTAGE = 3.3f;            // Volts
const int ADC_MAX_VALUE = 4095;                      // 12-bit ADC

// PID control parameters
const float INTEGRAL_DECAY_FACTOR = 0.996f;
const float INTEGRAL_ACCUMULATION_FACTOR = 0.004f;
const float MAX_INTEGRAL_ERROR = 50.0f;

// EMA filter gains (0.0 = no filtering, 1.0 = maximum filtering)
const float POSITION_EMA_GAIN = 0.5f;
const float VELOCITY_EMA_GAIN = 0.5f;
const float ACCELERATION_EMA_GAIN = 1.0f;  // No filtering
const float JERK_EMA_GAIN = 1.0f;          // No filtering
const float CURRENT_EMA_GAIN = 0.1f;       // Reduced for smoother current readings

// =============================================================================
// Motor State Namespace
// =============================================================================

namespace MotorState
{
    // Encoder state
    extern volatile long encoderPosition;
    extern volatile int lastEncoded;

    // Motor state variables (position, velocity, acceleration, jerk)
    extern float motorPos;        // Current position (radians)
    extern float motorVel;        // Current velocity (rad/s)
    extern float motorAcc;        // Current acceleration (rad/s²)
    extern float motorJrk;        // Current jerk (rad/s³)

    // Previous state for derivative calculations
    extern float lastMotorPos;    // Previous position
    extern float lastMotorVel;    // Previous velocity
    extern float lastMotorAcc;    // Previous acceleration
    extern float lastMotorJrk;    // Previous jerk

    // Current sensing
    extern float motorCurrent;    // Current motor current (A)
    extern float lastMotorCurrent; // Previous current reading

    // PID error terms
    extern float propError;       // Proportional error
    extern float intError;        // Integral error
    extern float derivError;      // Derivative error

    // Setpoint configuration
    extern float motorSetpoint;
    extern float setpointAmplitude;
    extern float setpointFrequency;

    // =============================================================================
    // State Management Functions
    // =============================================================================

    /**
     * Initialize motor state variables
     */
    void initialize();

    /**
     * Convert encoder counts to radians
     */
    float countsToRadians(long counts);

    /**
     * Apply exponential moving average filter
     */
    float applyEMAFilter(float currentValue, float previousValue, float gain);

    /**
     * Read current sensor and convert to amperes
     */
    float readCurrentSensor();

    /**
     * Run hardware self-test to verify sensor connections
     */
    void runHardwareSelfTest();

    /**
     * Update motor position with EMA filtering
     */
    void updateMotorPosition(long currentEncoderPosition);

    /**
     * Update motor velocity with EMA filtering
     */
    void updateMotorVelocity(float dt);

    /**
     * Update motor acceleration with EMA filtering
     */
    void updateMotorAcceleration(float dt);

    /**
     * Update motor jerk with EMA filtering
     */
    void updateMotorJerk(float dt);

    /**
     * Update current reading with EMA filtering
     */
    void updateMotorCurrent();

    /**
     * Update PID error terms
     */
    void updatePIDErrors();

    /**
     * Store current state as previous state for next iteration
     */
    void updatePreviousState();

    /**
     * Update sinusoidal setpoint
     */
    void updateSinusoidalSetpoint(float time, float dt);

    /**
     * Perform complete state update cycle
     */
    void updateAllStates(float dt);

    // =============================================================================
    // Interrupt Service Routine
    // =============================================================================

    /**
     * Encoder interrupt service routine
     */
    void encoderISR();
}

#endif // MOTOR_STATE_H