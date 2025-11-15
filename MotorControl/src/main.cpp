#include <Arduino.h>
#include <FreeRTOS_SAMD21.h>

#include "main.h"
#include "motorControl.h"
#include "PID.h"
#include <ArduinoEigenDense.h>

#include "RLAgent.h"

// PID control parameters
float kp = 10.0f;			// Proportional gain
float ki = 0.1f;			// Integral gain
float kd = 1.0f;			// Derivative gain
float maxIntegral = 100.0f; // Maximum integral windup

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

	delay(1000); // Allow time for setup stabilization

	xTaskCreate(TaskSystemIdentification, "SystemIDTask", 1024, NULL, 1, NULL);
	// xTaskCreate(TaskSensorPrints, "SensorPrintsTask", 512, NULL, 1, NULL);
	xTaskCreate(TaskSerialInput, "SerialInputTask", 256, NULL, 1, NULL);
	// xTaskCreate(TaskPIDControl, "PIDControlTask", 512, NULL, 2, NULL);
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

void TaskPIDControl(void *pvParameters)
{
	(void)pvParameters;
	Serial.println("PID Control task started!");

	const int taskFrequencyHz = 100;
	TickType_t delay = pdMS_TO_TICKS(1000 / taskFrequencyHz);
	float dt = 1.0f / taskFrequencyHz; // Time step in seconds

	for (;;)
	{
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

void TaskSystemIdentification(void *pvParameters)
{
	(void)pvParameters;

	const int taskFrequencyHz = 100;
	TickType_t delay = pdMS_TO_TICKS(1000 / taskFrequencyHz);
	Serial.println("System Identification task started!");
	int count = 0;

	using namespace SystemIdentification;
	initialize();
	MatrixXd input(model.NUMBER_OF_CONTROL, 1);
	for (;;)
	{
		TickType_t xLastWakeTime = xTaskGetTickCount();

		// Calculate time based on task iterations
		float timeSec = count * (1.0f / taskFrequencyHz);

		// Construct current state and input matrices
		MatrixXd currentState(model.NUMBER_OF_STATES, 1);
		float pos = isnan(MotorState::motorPos) ? 0.0f : MotorState::motorPos;
		float vel = isnan(MotorState::motorVel) ? 0.0f : MotorState::motorVel;
		float acc = isnan(MotorState::motorAcc) ? 0.0f : MotorState::motorAcc;
		float jrk = isnan(MotorState::motorJrk) ? 0.0f : MotorState::motorJrk;
		// Normalize states to [-1, 1] based on max values
		currentState(0, 0) = pos / MAX_POSITION;
		currentState(1, 0) = vel / MAX_VELOCITY;
		currentState(2, 0) = acc / MAX_ACCELERATION;
		currentState(3, 0) = jrk / MAX_JERK;

		// Generate a sine wave input
		float amplitude = 1.0f; // Normalized amplitude for learning
		float frequency = 0.5f; // Random frequency between 0.2 and 1.5 Hz
		float frequency2 = 0.33f;

		float motorSig = amplitude * sin(2 * PI * frequency * timeSec);
		motorSig += (amplitude / 2.0f) * sin(2 * PI * frequency2 * timeSec); // Add second sine component
		motorSig = constrain(motorSig, -1.0f, 1.0f);
		Motor::motorSpeed = static_cast<int>(motorSig * 255.0f);
		input(0, 0) = motorSig; // Use normalized input for learning

		// Update system model
		update(currentState, input);

		if (converged) {
			Serial.println("System identification converged!");
			model.printMatrices();
			vTaskSuspend(NULL);
		}

		count++;
		if (count >= 200) // Print every 1 second
		{
			model.printMatrices();
			Serial.println("Convergence Rate: " + String(currentUpdateNorm,6));
			Serial.println("Current Error: " + String(currentErrorNorm,6));
			Serial.println("Converged: " + String(converged ? "Yes" : "No"));
			count = 0;
		}

		vTaskDelayUntil(&xLastWakeTime, delay);
	}
}
