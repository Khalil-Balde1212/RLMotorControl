#include "main.h"
#include "../lib/motorControl/motorState.h"

void TaskSensorPrints(void *pvParameters)
{
    (void)pvParameters;
    int taskFrequencyHz = 10; // Reduced from 100Hz to 2Hz to reduce Serial load
    TickType_t delay = pdMS_TO_TICKS(1000 / taskFrequencyHz);

    for (;;)
    {
        TickType_t xLastWakeTime = xTaskGetTickCount();
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
        vTaskDelayUntil(&xLastWakeTime, delay);
    }
}

void TaskSerialInput(void *pvParameters)
{
    (void)pvParameters;

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
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}