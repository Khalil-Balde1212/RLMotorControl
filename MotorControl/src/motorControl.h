#include <Arduino.h>
#include <FreeRTOS_SAMD21.h>

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
// Encoder pins
#define ENCODER_A 2
#define ENCODER_B 3
#define MOTOR_PIN_A 5
#define MOTOR_PIN_B 6
#define CURRENT_SENSE_PIN A0

namespace Motor
{
    // Encoder variables
    extern volatile long encoderPosition;
    extern volatile int lastEncoded;
    
    // Motor state variables

    /// @brief Current motor position, velocity, acceleration, and jerk
    extern float motorPos; //theta
    extern float motorVel; //theta'
    extern float motorAcc; //theta''
    extern float motorJrk; //theta'''

    extern float lastMotorPos; //theta_{t-1}
    extern float lastMotorVel; //theta'_{t-1}
    extern float lastMotorAcc; //theta''_{t-1}
    extern float lastMotorJrk; //theta'''_{t-1}

    extern float motorPosEMAGain;
    extern float motorVelEMAGain;
    extern float motorAccEMAGain;
    extern float motorJrkEMAGain;

    extern float motorCurrent; // Current motor current
    extern float lastMotorCurrent;
    extern float motorCurrentEMAGain;
    
    /// @brief Counts per revolution for the encoder
    extern int countsPerRevolution; // Example value, set according to your encoder

    // Task to monitor and report encoder position
    void TaskSensorReads(void *pvParameters);
    void encoderISR();


    //Motor Variables
    extern int motorSpeed; // Example variable for motor speed control

    void TaskMotorControl(void *pvParameters);
    void TaskSerialInput(void *pvParameters);

    void SensorPrints(void *pvParameters);



    void setup();
}
#endif