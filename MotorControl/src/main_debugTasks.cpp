#include "main.h"


void TaskSensorPrints(void *pvParameters)
{
    (void)pvParameters;
    int taskFrequencyHz = 10;  // Reduced from 100Hz to 2Hz to reduce Serial load
    TickType_t delay = pdMS_TO_TICKS(1000 / taskFrequencyHz);

    for (;;)
    {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        Serial.print(Motor::motorPos);
        Serial.print(",");
        Serial.print(Motor::motorSetpoint);
        Serial.print(",");
        Serial.print(Motor::motorVel);
        Serial.print(",");
        Serial.print(Motor::motorAcc);
        Serial.print(",");
        Serial.print(Motor::motorJrk);
        Serial.print(",");
        Serial.print(Motor::motorCurrent);
        Serial.print(",");
        Serial.print(Motor::motorSpeed / 255.0f); // Print motor speed as percentage
        Serial.print(",");
        Serial.print(Motor::propError);    // P error
        Serial.print(",");
        Serial.print(Motor::intError);     // I error
        Serial.print(",");
        Serial.print(Motor::derivError);   // D error

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
            int speed = input.toInt();
            Motor::motorSpeed = speed;
            Serial.print("Motor speed set to: ");
            Serial.println(Motor::motorSpeed);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}