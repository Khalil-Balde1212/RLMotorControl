#include "main.h"
#include "../lib/motorControl/motorState.h"

void TaskSensorPrints()
{
    // No parameters for Mbed threads
    Serial.println("Sensor prints task started!");
    int taskFrequencyHz = 2; // Increased to 2Hz for debugging
    int delayMs = 1000 / taskFrequencyHz;

    for (;;)
    {
        Serial.print(MotorState::motorPos);
        Serial.print(",");
        Serial.print(MotorState::motorSetpoint);
        Serial.print(",");
        Serial.print(MotorState::motorVel);
        Serial.print(",");
        Serial.print(MotorState::motorAcc);
        Serial.print(",");
        Serial.print(MotorState::motorJrk);
        Serial.print(",");
        Serial.print(MotorState::motorCurrent);
        Serial.print(",");
        Serial.print(Motor::motorSpeed / 255.0f); // Print motor speed as percentage
        Serial.print(",");
        Serial.print(MotorState::propError); // P error
        Serial.print(",");
        Serial.print(MotorState::intError); // I error
        Serial.print(",");
        Serial.print(MotorState::derivError); // D error

        Serial.println();
        delay(delayMs);
    }
}

void TaskSerialInput()
{
    // No parameters for Mbed threads

    for (;;)
    {
        if (Serial.available())
        {
            String input = Serial.readStringUntil('\n');
            input.trim(); // Remove whitespace
            int speed = input.toInt();
            if (speed != 0 || input == "0")
            {
                Motor::motorSpeed = speed;
                Serial.print("Motor speed set to: ");
                Serial.println(Motor::motorSpeed);
            }
        }
    delay(10);
    }
}