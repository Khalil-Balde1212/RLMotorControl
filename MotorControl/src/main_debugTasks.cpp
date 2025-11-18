#include "main.h"
#include "../lib/motorControl/motorState.h"
#include "../lib/RLAgent/RLAgent.h"

void TaskSensorPrints()
{
    // No parameters for Mbed threads
    Serial.println("Sensor prints task started!");
    int taskFrequencyHz = 100;
    int delayMs = 1000 / taskFrequencyHz;

    for (;;)
    {
        Serial.print("SD,");
        Serial.print(MotorState::motorPos);
        Serial.print(",");
        Serial.print(MotorState::motorSetpoint);
        Serial.print(",");
        Serial.print(MotorState::motorVel);
        Serial.print(",");
        Serial.print(MotorState::motorAcc);
        Serial.print(",");
        Serial.print(MotorState::motorJrk);
        Serial.print(",");
        Serial.print(MotorState::motorCurrent);
        Serial.print(",");
        Serial.print(Motor::motorSpeed / 255.0f); // Print motor speed as percentage


        Serial.println();   
    if (useRLNN)
        {
            // Print cached predicted positions and RL reward
            std::vector<float> preds = RLPolicy::getCachedPos();
            Serial.print("RLPos,");
            for (size_t i = 0; i < preds.size(); ++i)
            {
                Serial.print(preds[i], 6);
                if (i + 1 < preds.size()) Serial.print(",");
            }
            Serial.print(",R:");
            Serial.println(RLPolicy::calculateReward(), 6);
            // print cached motor commands if any
            std::vector<int> rcmds = RLPolicy::getCachedCmds();
            Serial.print("RLCmds,");
            for (size_t i = 0; i < rcmds.size(); ++i)
            {
                Serial.print(rcmds[i]);
                if (i + 1 < rcmds.size()) Serial.print(",");
            }
            Serial.println();
        }
        delay(delayMs);
    }
}

void TaskSerialInput()
{
    // No parameters for Mbed threads

    for (;;)
    {
        if (Serial.available())
        {
            String input = Serial.readStringUntil('\n');
            input.trim(); // Remove whitespace
            int speed = input.toInt();
            if (speed != 0 || input == "0")
            {
                Motor::motorSpeed = speed;
                Serial.print("Motor speed set to: ");
                Serial.println(Motor::motorSpeed);
            }
            else if (input.equalsIgnoreCase("rl on"))
            {
                useRLNN = true;
                Serial.println("RL Neural policy enabled");
            }
            else if (input.equalsIgnoreCase("rl off"))
            {
                useRLNN = false;
                Serial.println("RL Neural policy disabled");
            }
            else if (input.equalsIgnoreCase("rl reward"))
            {
                float r = RLPolicy::calculateReward();
                Serial.print("RL Reward: ");
                Serial.println(r, 6);
            }
            else if (input.equalsIgnoreCase("rl dump"))
            {
                // dump motor cmd sequence
                rlMutex.lock();
                std::vector<int> seq = RLPolicy::getCachedCmds();
                rlMutex.unlock();
                Serial.print("RLCmds,");
                for (size_t i = 0; i < seq.size(); ++i)
                {
                    Serial.print(seq[i]);
                    if (i + 1 < seq.size()) Serial.print(",");
                }
                Serial.println();
            }
            else if (input.equalsIgnoreCase("rl learn on"))
            {
                RLLearning = true;
                Serial.println("RL learning enabled");
            }
            else if (input.equalsIgnoreCase("rl learn off"))
            {
                RLLearning = false;
                Serial.println("RL learning disabled");
            }
        }
    delay(10);
    }
}