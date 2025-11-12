#include <Arduino.h>
#include <FreeRTOS_SAMD21.h>
#include <BasicLinearAlgebra.h>
#include "motorControl.h"

using namespace BLA;

void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 5000)
  {
    ;
  }

  Motor::setup();
  Serial.println("Khalil sucks poopoo");
  Serial.println("This is Marc saying his opinion of Khalil");
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