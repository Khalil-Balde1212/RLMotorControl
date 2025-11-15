#ifndef MAIN_H
#define MAIN_H


#include "motorControl.h"

void setup();
void loop();

void TaskSensorPrints(void *pvParameters);
void TaskSerialInput(void *pvParameters);

void TaskPIDControl(void* pvParameters);
void TaskOscilateMotor(void* pvParameters);

void TaskCalibration(void *pvParameters);
void TaskSystemIdentification(void* pvParameters);

void TaskPeriodicModelUpdates(void* pvParameters);
#endif