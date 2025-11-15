#include <Arduino.h>
#include <FreeRTOS_SAMD21.h>

#include "main.h"
#include "motorControl.h"
#include "PID.h"
#include <ArduinoEigenDense.h>

#include "RLAgent.h"

// Task handles
TaskHandle_t systemIDTaskHandle;

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
	Serial.println("Setup started");
	while (!Serial && millis() < 5000)
	{
		;
	}
	Serial.println("Serial ready");

	Motor::setup();
	Serial.println("Motor setup done");

	randomSeed(analogRead(0)); // Seed random number generator
	Serial.println("Random seed set");

	delay(1000); // Allow time for setup stabilization
	Serial.println("Delay done");

	xTaskCreate(TaskCalibration, "CalibrationTask", 512, NULL, 1, NULL);
	xTaskCreate(TaskSystemIdentification, "SystemIDTask", 512, NULL, 1, &systemIDTaskHandle);
	vTaskSuspend(systemIDTaskHandle); // Suspend until calibration is done
	// xTaskCreate(TaskSensorPrints, "SensorPrintsTask", 512, NULL, 1, NULL);
	xTaskCreate(TaskSerialInput, "SerialInputTask", 256, NULL, 1, NULL);
	// xTaskCreate(TaskPIDControl, "PIDControlTask", 512, NULL, 2, NULL);
	Serial.println("Tasks created, starting scheduler");
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

void TaskCalibration(void *pvParameters) {
	(void)pvParameters;
	Serial.println("Starting motor calibration...");

	// Calibration: Sweep sine waves with increasing frequency to estimate limits
	float maxVel = 0, maxAcc = 0, maxJrk = 0;
	const float startFreq = 0.1f; // Hz
	const float endFreq = 10.0f;   // Hz
	const float sweepTime = 10.0f; // seconds
	const float amplitude = 255.0f; // Full range
	const int samples = 100; // Samples per sweep

	for (int i = 0; i < samples; i++) {
		float t = (float)i / (float)samples * sweepTime;
		float freq = startFreq + (endFreq - startFreq) * (float)i / (float)(samples - 1);
		float motorSig = amplitude * sin(2.0f * PI * freq * t);
		Motor::motorSpeed = (int)motorSig;
		
		vTaskDelay(pdMS_TO_TICKS(sweepTime * 1000 / samples));
		
		maxVel = max(maxVel, abs(MotorState::motorVel));
		maxAcc = max(maxAcc, abs(MotorState::motorAcc));
		maxJrk = max(maxJrk, abs(MotorState::motorJrk));
	}
	
	MAX_POSITION = 100.0f; // Assume known max position (radians)
	MAX_VELOCITY = maxVel;
	MAX_ACCELERATION = maxAcc;
	MAX_JERK = maxJrk;
	Motor::motorSpeed = 0;
	Serial.println("Calibration complete. MAX_VELOCITY: " + String(MAX_VELOCITY) + " MAX_ACC: " + String(MAX_ACCELERATION) + " MAX_JERK: " + String(MAX_JERK));

	// Resume system identification task
	vTaskResume(systemIDTaskHandle);
	vTaskSuspend(NULL); // Suspend calibration task
}

void TaskPIDControl(void* pvParameters)
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

	const int taskFrequencyHz = 50;
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
		static float timeSec = 0.0f;
		timeSec += 1.0f / taskFrequencyHz;

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

		// Generate a sine wave input with random amplitudes
		float amplitude = random(50, 100) / 100.0f; // Random amplitude 0.5 to 1.0
		// Chirp signal: frequency sweeps from 0.2 Hz to 1.5 Hz over time
		float chirpStartFreq = 0.2f;
		float chirpEndFreq = 1.5f;
		float chirpDuration = 30.0f; // seconds for full sweep
		float chirpFreq = chirpStartFreq + (chirpEndFreq - chirpStartFreq) * min(timeSec / chirpDuration, 1.0f);
		float frequency = chirpFreq;


		// Chirp signal for second frequency: sweeps from 0.33 Hz to 2.0 Hz over time
		float amplitude2 = random(30, 80) / 100.0f; // Random amplitude 0.3 to 0.8
		float chirp2StartFreq = 0.33f;
		float chirp2EndFreq = 2.0f;
		float chirp2Duration = 25.0f;
		float frequency2 = chirp2StartFreq + (chirp2EndFreq - chirp2StartFreq) * min(timeSec / chirp2Duration, 1.0f);
		float motorSig = amplitude * sin(2 * PI * frequency * timeSec);
		motorSig += (amplitude2 / 2.0f) * sin(2 * PI * frequency2 * timeSec);
		
		// Third chirp signal: sweeps from 0.5 Hz to 2.5 Hz over time
		float amplitude3 = random(20, 60) / 100.0f; // Random amplitude 0.2 to 0.6
		float chirp3StartFreq = 0.5f;
		float chirp3EndFreq = 2.5f;
		float chirp3Duration = 20.0f;
		float frequency3 = chirp3StartFreq + (chirp3EndFreq - chirp3StartFreq) * min(timeSec / chirp3Duration, 1.0f);
		motorSig += (amplitude3 / 3.0f) * sin(2 * PI * frequency3 * timeSec);


		motorSig = constrain(motorSig, -1.0f, 1.0f);
		Motor::motorSpeed = static_cast<int>(motorSig * 255.0f);
		input(0, 0) = motorSig; // Use normalized input for learning

		// Update system model
		update(currentState, input);
		// learningRate = 0.01f / (1.0f + currentUpdateNorm);  // Scales down as convergence slows
		learningRate *= 0.999f; // Gradually decrease learning rate
		if (converged) {
			Serial.println("System identification converged!");
			Serial.print("Time elapsed: ");
			Serial.print(timeSec, 2);
			Serial.println(" seconds");
			Motor::motorSpeed = 0;
			model.printMatrices();
			vTaskSuspend(NULL);
		}

		count++;
		if (count >= 10)
		{
			String csv = model.getMatricesCSV();
			csv += String(currentUpdateNorm, 6) + ",";
			csv += String(currentErrorNorm, 6) + ",";
			csv += String(converged ? 1 : 0);
			Serial.println(csv);
			count = 0;
		}

		vTaskDelayUntil(&xLastWakeTime, delay);
	}
}
