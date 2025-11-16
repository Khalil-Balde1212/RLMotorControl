#ifndef MAIN_H
#define MAIN_H


#include "motorControl.h"

void setup();
void loop();

void TaskSensorPrints();
void TaskSerialInput();

void TaskPIDControl();
void TaskOscilateMotor();

void TaskCalibration();
void TaskSystemIdentification();

void TaskPeriodicModelUpdates();
extern bool useRLNN;
#endif