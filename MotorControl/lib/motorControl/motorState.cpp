#include "motorState.h"
#include "motorControl.h"

// =============================================================================
// Namespace Variables
// =============================================================================

namespace MotorState
{
    // Encoder state
    volatile long encoderPosition = 0;
    volatile int lastEncoded = 0;

    // Motor state variables (position, velocity, acceleration, jerk)
    float motorPos = 0.0f;        // Current position (radians)
    float motorVel = 0.0f;        // Current velocity (rad/s)
    float motorAcc = 0.0f;        // Current acceleration (rad/s²)
    float motorJrk = 0.0f;        // Current jerk (rad/s³)

    // Previous state for derivative calculations
    float lastMotorPos = 0.0f;    // Previous position
    float lastMotorVel = 0.0f;    // Previous velocity
    float lastMotorAcc = 0.0f;    // Previous acceleration
    float lastMotorJrk = 0.0f;    // Previous jerk

    // Current sensing
    float motorCurrent = 0.0f;    // Current motor current (A)
    float lastMotorCurrent = 0.0f; // Previous current reading

    // PID error terms
    float propError = 0.0f;       // Proportional error
    float intError = 0.0f;        // Integral error
    float derivError = 0.0f;      // Derivative error

    // Setpoint configuration
    float motorSetpoint = 3.1415f; // Base target position (radians)
    float setpointAmplitude = 3.14f; // Amplitude for sinusoidal setpoint
    float setpointFrequency = 0.1f; // Frequency for sinusoidal setpoint
}

// =============================================================================
// State Management Functions
// =============================================================================

void MotorState::initialize()
{
    // Initialize all state variables to zero
    encoderPosition = 0;
    lastEncoded = 0;

    motorPos = 0.0f;
    motorVel = 0.0f;
    motorAcc = 0.0f;
    motorJrk = 0.0f;

    lastMotorPos = 0.0f;
    lastMotorVel = 0.0f;
    lastMotorAcc = 0.0f;
    lastMotorJrk = 0.0f;

    motorCurrent = 0.0f;
    lastMotorCurrent = 0.0f;

    propError = 0.0f;
    intError = 0.0f;
    derivError = 0.0f;

    motorSetpoint = 1.57f; // π/2 radians (90 degrees) - non-zero setpoint for testing
    setpointAmplitude = 3.14f;
    setpointFrequency = 0.1f;
}

float MotorState::countsToRadians(long counts) {
    return (2.0f * PI * counts) / ENCODER_COUNTS_PER_REVOLUTION;
}

float MotorState::applyEMAFilter(float currentValue, float previousValue, float gain) {
    return gain * currentValue + (1.0f - gain) * previousValue;
}

float MotorState::readCurrentSensor() {
    // Read ADC value and convert to voltage
    int adcValue = analogRead(CURRENT_SENSE_PIN);
    float voltage = adcValue * ADC_REFERENCE_VOLTAGE / ADC_MAX_VALUE;

    // Convert voltage to current using sensor characteristics
    float current = (voltage - CURRENT_SENSOR_VOLTAGE_CENTER) / CURRENT_SENSOR_SENSITIVITY;

    return current;
}

void MotorState::updateMotorPosition(long currentEncoderPosition) {
    float rawPos = countsToRadians(currentEncoderPosition);
    motorPos = applyEMAFilter(rawPos, lastMotorPos, POSITION_EMA_GAIN);
}

void MotorState::updateMotorVelocity(float dt) {
    float rawVel = (motorPos - lastMotorPos) / dt;
    motorVel = applyEMAFilter(rawVel, lastMotorVel, VELOCITY_EMA_GAIN);
}

void MotorState::updateMotorAcceleration(float dt) {
    float rawAcc = (motorVel - lastMotorVel) / dt;
    motorAcc = applyEMAFilter(rawAcc, lastMotorAcc, ACCELERATION_EMA_GAIN);
}

void MotorState::updateMotorJerk(float dt) {
    float rawJrk = (motorAcc - lastMotorAcc) / dt;
    motorJrk = applyEMAFilter(rawJrk, lastMotorJrk, JERK_EMA_GAIN);
}

void MotorState::updateMotorCurrent() {
    float rawCurrent = readCurrentSensor();
    motorCurrent = applyEMAFilter(rawCurrent, lastMotorCurrent, CURRENT_EMA_GAIN);
}

void MotorState::updatePIDErrors() {
    // Proportional error: difference between setpoint and position
    propError = motorSetpoint - motorPos;

    // Derivative error: negative velocity (approximation for slowly changing setpoints)
    derivError = -motorVel;

    // Integral error: decayed accumulation with windup protection
    intError = intError * INTEGRAL_DECAY_FACTOR +
               propError * INTEGRAL_ACCUMULATION_FACTOR;
    intError = constrain(intError, -MAX_INTEGRAL_ERROR, MAX_INTEGRAL_ERROR);
}

void MotorState::updatePreviousState() {
    lastMotorPos = motorPos;
    lastMotorVel = motorVel;
    lastMotorAcc = motorAcc;
    lastMotorJrk = motorJrk;
    lastMotorCurrent = motorCurrent;
}

void MotorState::updateAllStates(float dt) {
    // Read current encoder position (atomic operation)
    long currentPosition = encoderPosition;

    // Update all motor state variables
    updateMotorPosition(currentPosition);
    updateMotorVelocity(dt);
    updateMotorAcceleration(dt);
    updateMotorJerk(dt);
    updateMotorCurrent();

    // Update PID error calculations
    updatePIDErrors();

    // Store current state for next derivative calculations
    updatePreviousState();
}

// =============================================================================
// Interrupt Service Routine
// =============================================================================

void MotorState::encoderISR()
{
    // Read encoder pins
    int MSB = digitalRead(ENCODER_A); // Most significant bit
    int LSB = digitalRead(ENCODER_B); // Least significant bit

    // Combine bits into encoded value
    int encoded = (MSB << 1) | LSB;

    // Calculate transition pattern
    int sum = (lastEncoded << 2) | encoded;

    // Determine rotation direction based on quadrature encoding
    // Clockwise transitions
    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
        encoderPosition++;
    }
    // Counter-clockwise transitions
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
        encoderPosition--;
    }

    // Store current state for next interrupt
    lastEncoded = encoded;
}
