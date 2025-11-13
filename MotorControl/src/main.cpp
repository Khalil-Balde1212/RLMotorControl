#include <Arduino.h>
#include <FreeRTOS_SAMD21.h>
#include <BasicLinearAlgebra.h>

#include "main.h"
#include "motorControl.h"
#include "PID.h"

using namespace BLA;

void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 5000)
  {
    ;
  }

  Motor::setup();
  
  xTaskCreate(TaskSensorPrints, "SensorPrintsTask", 512, NULL, 1, NULL);
  xTaskCreate(TaskSerialInput, "SerialInputTask", 256, NULL, 1, NULL);
  xTaskCreate(TaskPIDControl, "PIDControlTask", 512, NULL, 2, NULL);
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


void TaskPIDControl(void* pvParameters) {
    (void)pvParameters;
    // PD Controller - reduced gains for slower oscillations
    // Ku = 850, but oscillations too fast, so using 10% of Ku
    const float kp = 540.0f;     // Reduced proportional gain (10% of Ku)
    const float ki = 10.0f;      // No integral
    const float kd = 1.22f;     // Small derivative damping
    const float maxIntegral = 100.0f;

    float integral = 0.0f;
    float previousError = 0.0f;
    const int taskFrequencyHz = 100;
    TickType_t delay = pdMS_TO_TICKS(1000 / taskFrequencyHz);

    for (;;) {
        TickType_t xLastWakeTime = xTaskGetTickCount();

        // Get current setpoint and measured position
        float setpoint = MotorState::motorSetpoint;
        float measured = MotorState::motorPos;

        // Compute PID output
        int pidOutput = computePID(setpoint, measured, integral, previousError,
                                   kp, ki, kd, 1.0f / taskFrequencyHz, maxIntegral);

        // Update motor speed command
        Motor::motorSpeed = constrain(pidOutput, -255, 255);

        vTaskDelayUntil(&xLastWakeTime, delay);
    }
}





