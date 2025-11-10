#include "motorControl.h"

namespace Motor
{
    volatile long encoderPosition = 0;
    volatile int lastEncoded = 0;
    int countsPerRevolution = 1940;

    int motorSpeed = 0;
}

void Motor::setup()
{
    // Initialize encoder pins
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR, CHANGE);

    // Initialize motor pins
    pinMode(MOTOR_PIN_A, OUTPUT);
    pinMode(MOTOR_PIN_B, OUTPUT);

    // Create Encoder Task
    xTaskCreate(TaskEncoder, "EncoderTask", 256, NULL, 1, NULL);
    xTaskCreate(TaskMotorControl, "MotorControlTask", 256, NULL, 1, NULL);
    xTaskCreate(TaskSerialInput, "SerialInputTask", 256, NULL, 1, NULL);
}

void Motor::encoderISR()
{
    int MSB = digitalRead(ENCODER_A); // Most significant bit
    int LSB = digitalRead(ENCODER_B); // Least significant bit

    int encoded = (MSB << 1) | LSB;         // Convert the 2 pin value to single number
    int sum = (lastEncoded << 2) | encoded; // Combine previous and current

    // Determine direction
    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        encoderPosition++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        encoderPosition--;

    lastEncoded = encoded; // Store current for next interrupt
}

void Motor::TaskEncoder(void *pvParameters)
{
    (void)pvParameters;
    // long lastPosition = 0;

    for (;;)
    {
        long currentPosition = encoderPosition;
        motorPos = (2.0f * 3.14159265f * currentPosition) / countsPerRevolution;
        motorPos = motorPosEMAGain * motorPos + (1.0f - motorPosEMAGain) * lastMotorPos;
        lastMotorPos = motorPos;


        motorVel = (motorPos - lastMotorPos) / (0.1f); // Assuming 100ms interval
        motorVel = motorVelEMAGain * motorVel + (1.0f - motorVelEMAGain) * motorVel;
        // Serial.println(currentPosition);


        TickType_t xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);
    }
}

void Motor::TaskMotorControl(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        if(motorSpeed > 0)
        {
            analogWrite(MOTOR_PIN_A, motorSpeed);
            analogWrite(MOTOR_PIN_B, 0);
        }
        else
        if(motorSpeed < 0){
            analogWrite(MOTOR_PIN_A, 0);
            analogWrite(MOTOR_PIN_B, -motorSpeed);
        }
        else
        {
            analogWrite(MOTOR_PIN_A, 0);
            analogWrite(MOTOR_PIN_B, 0);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void Motor::TaskSerialInput(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        if (Serial.available())
        {
            String input = Serial.readStringUntil('\n');
            int speed = input.toInt();
            Motor::motorSpeed = speed;
            Serial.print("Motor speed set to: ");
            Serial.println(Motor::motorSpeed);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

