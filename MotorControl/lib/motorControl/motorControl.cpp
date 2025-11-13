#include "motorControl.h"

namespace Motor
{
    volatile long encoderPosition = 0;
    volatile int lastEncoded = 0;
    int countsPerRevolution = 1940;
    float motorSetpoint = 3.1415f; // Base target position (radians)
    float setpointAmplitude = 3.14f; // Reduced amplitude for easier tracking
    float setpointFrequency = 0.1f; // Increased frequency for more dynamic learning

    float motorPos = 0.0f; //theta
    float motorVel = 0.0f; //theta'
    float motorAcc = 0.0f; //theta''
    float motorJrk = 0.0f; //theta'''
    float lastMotorPos = 0.0f; //theta_{t-1}
    float lastMotorVel = 0.0f; //theta'_{t-1}
    float lastMotorAcc = 0.0f; //theta''_{t-1}
    float lastMotorJrk = 0.0f; //theta'''_{t-1}

    float motorPosEMAGain = 0.5f;
    float motorVelEMAGain = 0.5f;
    float motorAccEMAGain = 1.0f;   // No filtering for acceleration
    float motorJrkEMAGain = 1.0f;   // No filtering for jerk

    float motorCurrent = 0.0f; // Current motor current
    float lastMotorCurrent = 0.0f;
    float motorCurrentEMAGain = 0.1f;

    // PID error terms
    float propError = 0.0f;     // Proportional error
    float intError = 0.0f;      // Integral error
    float derivError = 0.0f;    // Derivative error

    int motorSpeed = 0;
}

void Motor::setup()
{
    // Initialize encoder pins
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR, CHANGE);

    // Initialize motor pins
    pinMode(MOTOR_PIN_A, OUTPUT);
    pinMode(MOTOR_PIN_B, OUTPUT);
    pinMode(CURRENT_SENSE_PIN, INPUT);

    analogReadResolution(12); // 12-bit ADC resolution

    // Create Encoder Task
    xTaskCreate(TaskSensorReads, "EncoderTask", 512, NULL, 1, NULL);
    xTaskCreate(TaskMotorControl, "MotorControlTask", 256, NULL, 1, NULL);
}

void Motor::encoderISR()
{
    int MSB = digitalRead(ENCODER_A); // Most significant bit
    int LSB = digitalRead(ENCODER_B); // Least significant bit

    int encoded = (MSB << 1) | LSB;         // Convert the 2 pin value to single number
    int sum = (lastEncoded << 2) | encoded; // Combine previous and current

    // Determine direction
    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        encoderPosition++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        encoderPosition--;

    lastEncoded = encoded; // Store current for next interrupt
}

void Motor::TaskSensorReads(void *pvParameters)
{
    (void)pvParameters;
    int taskFrequencyHz = 100;  // Reduced from 1000Hz to reduce CPU load
    TickType_t delay = pdMS_TO_TICKS(1000 / taskFrequencyHz);
    // long lastPosition = 0;

    static float time = 0.0f; // Time in seconds

    for (;;)
    {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        long currentPosition = encoderPosition;
        float rawPos = (2.0f * 3.14159265f * currentPosition) / countsPerRevolution; // Convert counts to radians
        motorPos = motorPosEMAGain * rawPos + (1.0f - motorPosEMAGain) * lastMotorPos;
        
        // Update sinusoidal setpoint
        time += 0.01f; // Increment time (10ms per iteration at 100Hz)
        float baseSetpoint = 3.1415f;
        motorSetpoint = baseSetpoint + setpointAmplitude * sin(2.0f * 3.14159265f * setpointFrequency * time);
        
        float dt = 0.01f; // 10ms at 100Hz
        float rawVel = (motorPos - lastMotorPos) / dt;
        motorVel = motorVelEMAGain * rawVel + (1.0f - motorVelEMAGain) * lastMotorVel;

        float rawAcc = (motorVel - lastMotorVel) / dt;
        motorAcc = motorAccEMAGain * rawAcc + (1.0f - motorAccEMAGain) * lastMotorAcc;

        float rawJrk = (motorAcc - lastMotorAcc) / dt;
        motorJrk = motorJrkEMAGain * rawJrk + (1.0f - motorJrkEMAGain) * lastMotorJrk;

        // ACS712 5A: 185mV/A, centered at 2.5V (before divider)
        // With 1/2 voltage divider: centered at 1.25V, sensitivity 92.5mV/A
        float voltage = analogRead(CURRENT_SENSE_PIN) * 3.3f / 4095.0f; // Convert ADC to voltage (3.3V supply, 12-bit ADC)
        float rawCurrent = (voltage - 1.25f) / 0.0925f; // Convert voltage to current (92.5mV/A = 0.0925V/A)
        motorCurrent = motorCurrentEMAGain * rawCurrent + (1.0f - motorCurrentEMAGain) * lastMotorCurrent;

        // Compute PID error terms
        propError = motorSetpoint - motorPos;                    // P: setpoint - position
        derivError = -motorVel;                                  // D: -velocity (approximation when setpoint changes slowly)
        intError = intError * 0.996f + propError * 0.004f;       // I: decayed accumulation (adjusted for 100Hz)
        intError = constrain(intError, -50.0f, 50.0f);          // Cap integral windup

        lastMotorPos = motorPos;
        lastMotorVel = motorVel;
        lastMotorAcc = motorAcc;
        lastMotorJrk = motorJrk;
        lastMotorCurrent = motorCurrent;

        vTaskDelayUntil(&xLastWakeTime, delay);
    }
}

void Motor::TaskMotorControl(void *pvParameters)
{
    (void)pvParameters;
    int taskFrequencyHz = 100;  // Reduced from 1000Hz
    TickType_t delay = pdMS_TO_TICKS(1000 / taskFrequencyHz);

    for (;;)
    {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        if(motorSpeed > 0)
        {
            analogWrite(MOTOR_PIN_A, motorSpeed);
            analogWrite(MOTOR_PIN_B, 0);
        }
        else
        if(motorSpeed < 0){
            analogWrite(MOTOR_PIN_A, 0);
            analogWrite(MOTOR_PIN_B, -motorSpeed);
        }
        else
        {
            analogWrite(MOTOR_PIN_A, 0);
            analogWrite(MOTOR_PIN_B, 0);
        }
        vTaskDelayUntil(&xLastWakeTime, delay);
    }
}



