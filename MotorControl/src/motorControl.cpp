#include "motorControl.h"

namespace Motor
{
    volatile long encoderPosition = 0;
    volatile int lastEncoded = 0;
    int countsPerRevolution = 1940;

    float motorPos = 0.0f; //theta
    float motorVel = 0.0f; //theta'
    float motorAcc = 0.0f; //theta''
    float motorJrk = 0.0f; //theta'''
    float lastMotorPos = 0.0f; //theta_{t-1}
    float lastMotorVel = 0.0f; //theta'_{t-1}
    float lastMotorAcc = 0.0f; //theta''_{t-1}
    float lastMotorJrk = 0.0f; //theta'''_{t-1}

    float motorPosEMAGain = 0.1f;
    float motorVelEMAGain = 0.1f;
    float motorAccEMAGain = 0.1f;
    float motorJrkEMAGain = 0.1f;

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

    //serial tasks
    xTaskCreate(TaskSerialInput, "SerialInputTask", 256, NULL, 1, NULL);
    xTaskCreate(TaskEncoderPrints, "EncoderPrintsTask", 256, NULL, 1, NULL);
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
    int taskFrequencyHz = 100;
    TickType_t delay = pdMS_TO_TICKS(1000 / taskFrequencyHz);
    // long lastPosition = 0;

    for (;;)
    {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        long currentPosition = encoderPosition;
        float rawPos = (2.0f * 3.14159265f * currentPosition) / countsPerRevolution; // Convert counts to radians
        motorPos = motorPosEMAGain * rawPos + (1.0f - motorPosEMAGain) * lastMotorPos;
        
        float rawVel = (motorPos - lastMotorPos) / (0.01f);
        motorVel = motorVelEMAGain * rawVel + (1.0f - motorVelEMAGain) * lastMotorVel;

        float rawAcc = (motorVel - lastMotorVel) / (0.01f);
        motorAcc = motorAccEMAGain * rawAcc + (1.0f - motorAccEMAGain) * lastMotorAcc;

        float rawJrk = (motorAcc - lastMotorAcc) / (0.01f);
        motorJrk = motorJrkEMAGain * rawJrk + (1.0f - motorJrkEMAGain) * lastMotorJrk;

        lastMotorPos = motorPos;
        lastMotorVel = motorVel;
        lastMotorAcc = motorAcc;
        lastMotorJrk = motorJrk;

        vTaskDelayUntil(&xLastWakeTime, delay);
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

void Motor::TaskEncoderPrints(void *pvParameters)
{
    (void)pvParameters;
    int taskFrequencyHz = 10;
    TickType_t delay = pdMS_TO_TICKS(1000 / taskFrequencyHz);

    for (;;)
    {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        Serial.print(motorPos);
        Serial.print(",");
        Serial.print(motorVel);
        Serial.print(",");
        Serial.print(motorAcc);
        Serial.print(",");
        Serial.println(motorJrk);
        vTaskDelayUntil(&xLastWakeTime, delay);
    }
}

