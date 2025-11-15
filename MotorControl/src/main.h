#ifndef MAIN_H
#define MAIN_H


#include "motorControl.h"

void setup();
void loop();

void TaskSensorPrints(void *pvParameters);
void TaskSerialInput(void *pvParameters);
void TaskPIDControl(void* pvParameters);


void TaskSystemIdentification(void* pvParameters);
#endif