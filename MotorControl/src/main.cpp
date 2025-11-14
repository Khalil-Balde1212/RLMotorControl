#include <Arduino.h>
#include <FreeRTOS_SAMD21.h>

#include "main.h"
#include "motorControl.h"
#include "PID.h"

// PID control parameters
float kp = 10.0f;    // Proportional gain
float ki = 0.1f;     // Integral gain
float kd = 1.0f;     // Derivative gain
float maxIntegral = 100.0f;  // Maximum integral windup

// PID state variables
float integral = 0.0f;
float previousError = 0.0f;

void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 5000)
  {
    ;
  }

  Motor::setup();

  Serial.println("Starting PID Motor Control...");
  Serial.print("Target setpoint: ");
  Serial.println(MotorState::motorSetpoint, 3);
  Serial.print("PID gains: Kp=");
  Serial.print(kp);
  Serial.print(" Ki=");
  Serial.print(ki);
  Serial.print(" Kd=");
  Serial.println(kd);
  delay(1000); // Give time for serial to be ready

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
    Serial.println("PID Control task started!");

    const int taskFrequencyHz = 100;
    TickType_t delay = pdMS_TO_TICKS(1000 / taskFrequencyHz);
    float dt = 1.0f / taskFrequencyHz; // Time step in seconds

    for (;;) {
        TickType_t xLastWakeTime = xTaskGetTickCount();

        // Get current position
        float currentPosition = MotorState::motorPos;

        // Compute PID control action
        int motorCommand = computePID(MotorState::motorSetpoint, currentPosition,
                                    integral, previousError, kp, ki, kd, dt, maxIntegral);

        // Constrain to motor limits
        motorCommand = constrain(motorCommand, -255, 255);

        // Apply control to motor
        Motor::motorSpeed = motorCommand;

        vTaskDelayUntil(&xLastWakeTime, delay);
    }
}





