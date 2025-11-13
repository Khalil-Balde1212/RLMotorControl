#include <Arduino.h>
#include <FreeRTOS_SAMD21.h>
#include <BasicLinearAlgebra.h>

#include "main.h"
#include "motorControl.h"
#include "tinyml.h"

using namespace BLA;

void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 5000)
  {
    ;
  }

  Motor::setup();
  // TinyML::setup();

  xTaskCreate(TaskSensorPrints, "SensorPrintsTask", 512, NULL, 1, NULL);
  vTaskStartScheduler();

  // If we get here, there was insufficient memory to create idle task
  Serial.println("ERROR: Scheduler failed to start!");
  while (1)
    ;
}

void loop()
{
  // Empty - all work done in FreeRTOS tasks
}


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
        Serial.print(",");
        Serial.println(TinyML::reward);

        vTaskDelayUntil(&xLastWakeTime, delay);
    }
}

