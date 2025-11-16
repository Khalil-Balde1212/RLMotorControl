#ifndef MAIN_H
#define MAIN_H


#include "motorControl.h"
#include <mbed.h>

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
void TaskRLControl();
void TaskRLApply();
extern bool RLLearning; // Toggle for policy weight updates
extern rtos::Mutex rlMutex;
#endif