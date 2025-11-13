#include "tinyml.h"
#include <Arduino.h>
#include <FreeRTOS_SAMD21.h>
#include "motorControl.h"

// Define extern variables
float TinyML::weights[NUM_FEATURES];
int TinyML::action = 0;
float TinyML::reward = 0.0f;

void TinyML::setup()
{
    randomSeed(analogRead(0));
    initializeWeights();

    xTaskCreate(TinyML::TaskTinyML, "TinyMLTask", 512, NULL, 1, NULL);
}

void TinyML::initializeWeights()
{
    // Initialize with oscillating behavior for better exploration
    // Create a policy that oscillates based on position error
    weights[0] = 0.0f;  // bias
    weights[1] = 50.0f; // strong response to position error (towards setpoint)
    weights[2] = -10.0f; // dampen velocity
    weights[3] = 0.0f;  // acceleration
    weights[4] = 0.0f;  // jerk
    weights[5] = 0.0f;  // current
    weights[6] = -20.0f; // accumulated error (integral term)
}

float TinyML::computeAction(float pos, float vel, float acc, float jrk, float curr, float accumErr)
{
    // Normalize features to [-1, 1]
    float normPos = pos / POS_MAX;
    float normVel = vel / VEL_MAX;
    float normAcc = acc / ACC_MAX;
    float normJrk = jrk / JRK_MAX;
    float normCurr = curr / CURR_MAX;
    float normErr = accumErr / ERR_MAX;

    // Linear policy: action = w0*1 + w1*pos + w2*vel + w3*acc + w4*jrk + w5*curr + w6*err
    float action = weights[0] * 1.0f +
                   weights[1] * normPos +
                   weights[2] * normVel +
                   weights[3] * normAcc +
                   weights[4] * normJrk +
                   weights[5] * normCurr +
                   weights[6] * normErr;

    // Constrain to motor speed limits
    action = constrain(action, -255.0f, 255.0f);

    return action;
}

float TinyML::computeValue(float pos, float vel, float acc, float jrk, float curr, float accumErr)
{
    // For continuous actions, we can use the same function or a separate value function
    // For simplicity, we'll use the action magnitude as a proxy for value
    return abs(computeAction(pos, vel, acc, jrk, curr, accumErr));
}

void TinyML::updateWeights(float oldPos, float oldVel, float oldAcc, float oldJrk, float oldCurr, float oldAccumErr, float action, float reward, float newPos, float newVel, float newAcc, float newJrk, float newCurr, float newAccumErr)
{
    // Compute value for old state
    float oldValue = computeValue(oldPos, oldVel, oldAcc, oldJrk, oldCurr, oldAccumErr);

    // Compute value for new state
    float newValue = computeValue(newPos, newVel, newAcc, newJrk, newCurr, newAccumErr);

    // TD target
    float target = reward + GAMMA * newValue;

    // TD error
    float tdError = target - oldValue;

    // For continuous actions, we update the policy weights based on TD error
    // Update weights with TD error (positive reinforcement for good actions)
    // Normalize features for gradient
    float normPos = oldPos / POS_MAX;
    float normVel = oldVel / VEL_MAX;
    float normAcc = oldAcc / ACC_MAX;
    float normJrk = oldJrk / JRK_MAX;
    float normCurr = oldCurr / CURR_MAX;
    float normErr = oldAccumErr / ERR_MAX;

    // Update weights - use TD error as learning signal
    weights[0] += ALPHA * tdError * 1.0f;
    weights[1] += ALPHA * tdError * normPos;
    weights[2] += ALPHA * tdError * normVel;
    weights[3] += ALPHA * tdError * normAcc;
    weights[4] += ALPHA * tdError * normJrk;
    weights[5] += ALPHA * tdError * normCurr;
    weights[6] += ALPHA * tdError * normErr;
}

float TinyML::computeReward(float pos, float vel, float acc, float jrk, float curr)
{
    float reward = 0.0f;
    reward -= abs(pos - Motor::motorSetpoint) * 0.5f;
    // reward -= abs(vel) * 0.1f;
    // reward -= abs(jrk) * 0.001f;

    if (pos > POS_MAX || pos < 0.0f) {
        reward -= 2.0f;
    }

    // if (pos > Motor::motorSetpoint && vel > 0) {
    //     reward -= 1.0f;
    // }
    // if (pos < Motor::motorSetpoint && vel < 0) {
    //     reward -= 1.0f;
    // }

    if (abs(vel) < 0.1f && abs(pos - Motor::motorSetpoint) < 0.1f) {
        reward += 20.0f;
    }

    return reward;
}

void TinyML::TaskTinyML(void *pvParameters)
{
    (void)pvParameters;
    int taskFrequencyHz = 10;
    TickType_t delay = pdMS_TO_TICKS(1000 / taskFrequencyHz);

    // Store previous state for update
    static float prevPos = 0.0f;
    static float prevVel = 0.0f;
    static float prevAcc = 0.0f;
    static float prevJrk = 0.0f;
    static float prevCurr = 0.0f;
    static float prevAccumErr = 0.0f;
    static float prevAction = 0.0f;

    // Accumulated position error
    static float accumErr = 0.0f;

    for (;;)
    {
        TickType_t xLastWakeTime = xTaskGetTickCount();

        // Get current state
        float currPos = Motor::motorPos;
        float currVel = Motor::motorVel;
        float currAcc = Motor::motorAcc;
        float currJrk = Motor::motorJrk;
        float currCurr = Motor::motorCurrent;

        // Accumulate position error
        float posError = abs(currPos - Motor::motorSetpoint);
        accumErr += posError * 0.1f; // integrate error over time (dt = 0.1s)
        accumErr = constrain(accumErr, 0.0f, ERR_MAX); // cap at max

        float chosenAction = computeAction(currPos, currVel, currAcc, currJrk, currCurr, accumErr);

        Motor::motorSpeed = (int)chosenAction;

        if ((Motor::motorPos > POS_MAX && Motor::motorSpeed > 0) ||
            (Motor::motorPos < -POS_MAX && Motor::motorSpeed < 0)) {
            Motor::motorSpeed = 0;
        }

        float reward = computeReward(currPos, currVel, currAcc, currJrk, currCurr);

        // Serial.print("Action: ");
        // Serial.print(chosenAction);
        // Serial.print(" Speed: ");
        // Serial.print(Motor::motorSpeed);
        // Serial.print(" AccumErr: ");
        // Serial.print(accumErr);
        // Serial.print(" Reward: ");
        // Serial.println(reward);

        // Update weights using previous state and current reward
        if (prevPos != 0.0f || prevVel != 0.0f || prevAcc != 0.0f) { // Skip first iteration
            updateWeights(prevPos, prevVel, prevAcc, prevJrk, prevCurr, prevAccumErr, prevAction, reward, currPos, currVel, currAcc, currJrk, currCurr, accumErr);
        }

        // Store current state as previous for next iteration
        prevPos = currPos;
        prevVel = currVel;
        prevAcc = currAcc;
        prevJrk = currJrk;
        prevCurr = currCurr;
        prevAccumErr = accumErr;
        prevAction = chosenAction;

        vTaskDelayUntil(&xLastWakeTime, delay);
    }
}