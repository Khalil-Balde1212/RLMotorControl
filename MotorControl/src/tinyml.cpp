#include "tinyml.h"
#include <Arduino.h>
#include <FreeRTOS_SAMD21.h>
#include "motorControl.h"
#include <cmath>

// Define extern variables
float TinyML::weights[NUM_FEATURES];
int TinyML::action = 0;
float TinyML::reward = 0.0f;
float TinyML::avgTDError = 0.0f; // Shared TD error average
int TinyML::exploreActionCount = 0; // Track exploration actions for metrics
static float lastTDError = 0.0f; // Last computed TD error

// Define queue handle outside namespace
QueueHandle_t experienceQueue;

void TinyML::setup()
{
    randomSeed(analogRead(0));
    initializeWeights();

    // Create experience queue for inter-task communication (increased size)
    experienceQueue = xQueueCreate(20, sizeof(Experience)); // Buffer up to 20 experiences

    xTaskCreate(TinyML::TaskActionSelection, "ActionTask", 512, NULL, 2, NULL); // Higher priority
    xTaskCreate(TinyML::TaskLearning, "LearningTask", 512, NULL, 1, NULL);     // Lower priority
}

void TinyML::initializeWeights()
{
    // Initialize with small weights for stable learning
    // Start with small random values around zero to avoid large initial actions
    randomSeed(analogRead(0)); // Seed random number generator

    // Small initial weights to prevent instability
    weights[0] = 0.1f;     // bias (small base action)
    weights[1] = 0.5f;     // position (moderate influence)
    weights[2] = -0.5f;    // setpoint (negative to drive toward setpoint)
    weights[3] = -0.2f;    // velocity (light damping)
    weights[4] = 0.01f;    // acceleration (minimal influence)
    weights[5] = 0.001f;   // jerk (very small influence)
    weights[6] = 0.05f;    // current (small influence)
    weights[7] = 1.0f;     // motor speed (primary control)
    weights[8] = 2.0f;     // P error (strong proportional control)
    weights[9] = -0.5f;    // I error (moderate integral control)
    weights[10] = -0.3f;   // D error (light derivative control)
    weights[11] = 0.0f;    // unused (was reward)

    // Add small random noise to break symmetry
    for (int i = 0; i < NUM_FEATURES; i++) {
        weights[i] += (random(-100, 101) / 1000.0f); // Add ±0.1 random noise
    }
}

float TinyML::computeAction(float pos, float setpoint, float vel, float acc, float jrk, float curr, float speed, float pErr, float iErr, float dErr)
{
    // Normalize all 11 features to [-1, 1] range
    float normPos = constrain(pos / POS_MAX, -1.0f, 1.0f);
    float normSetpoint = constrain(setpoint / SETPOINT_MAX, -1.0f, 1.0f);
    float normVel = constrain(vel / VEL_MAX, -1.0f, 1.0f);
    float normAcc = constrain(acc / ACC_MAX, -1.0f, 1.0f);
    float normJrk = constrain(jrk / JRK_MAX, -1.0f, 1.0f);
    float normCurr = constrain(curr / CURR_MAX, -1.0f, 1.0f);
    float normSpeed = constrain(speed / SPEED_MAX, -1.0f, 1.0f);
    float normPErr = constrain(pErr / P_ERR_MAX, -1.0f, 1.0f);
    float normIErr = constrain(iErr / I_ERR_MAX, -1.0f, 1.0f);
    float normDErr = constrain(dErr / D_ERR_MAX, -1.0f, 1.0f);

    // Linear policy: action = sum of (weight_i * normalized_feature_i)
    float action = weights[0] * 1.0f +           // bias
                   weights[1] * normPos +         // position
                   weights[2] * normSetpoint +    // setpoint
                   weights[3] * normVel +         // velocity
                   weights[4] * normAcc +         // acceleration
                   weights[5] * normJrk +         // jerk
                   weights[6] * normCurr +        // current
                   weights[7] * normSpeed +       // motor speed
                   weights[8] * normPErr +        // P error
                   weights[9] * normIErr +        // I error
                   weights[10] * normDErr;        // D error

    // Check for NaN or inf
    if (isnan(action) || isinf(action)) {
        action = 0.0f; // Fallback to safe action
    }

    // Constrain to motor speed limits (-255 to 255)
    action = constrain(action, -255.0f, 255.0f);

    return action;
}

float TinyML::computeValue(float pos, float setpoint, float vel, float acc, float jrk, float curr, float speed, float pErr, float iErr, float dErr)
{
    // For continuous actions, we can use the same function or a separate value function
    // For simplicity, we'll use the action magnitude as a proxy for value
    return abs(computeAction(pos, setpoint, vel, acc, jrk, curr, speed, pErr, iErr, dErr));
}

void TinyML::updateWeights(float oldPos, float oldSetpoint, float oldVel, float oldAcc, float oldJrk, float oldCurr, float oldSpeed, float oldPErr, float oldIErr, float oldDErr, float action, float reward, float newPos, float newSetpoint, float newVel, float newAcc, float newJrk, float newCurr, float newSpeed, float newPErr, float newIErr, float newDErr)
{
    // Compute value for old state
    float oldValue = computeValue(oldPos, oldSetpoint, oldVel, oldAcc, oldJrk, oldCurr, oldSpeed, oldPErr, oldIErr, oldDErr);

    // Compute value for new state
    float newValue = computeValue(newPos, newSetpoint, newVel, newAcc, newJrk, newCurr, newSpeed, newPErr, newIErr, newDErr);

    // TD target
    float target = reward + GAMMA * newValue;

    // TD error
    float tdError = target - oldValue;

    // More aggressive TD error clipping for stability
    tdError = constrain(tdError, -5.0f, 5.0f);

    // Store TD error for adaptive exploration
    lastTDError = abs(tdError);

    // Normalize features for gradient computation
    float normPos = constrain(oldPos / POS_MAX, -1.0f, 1.0f);
    float normSetpoint = constrain(oldSetpoint / SETPOINT_MAX, -1.0f, 1.0f);
    float normVel = constrain(oldVel / VEL_MAX, -1.0f, 1.0f);
    float normAcc = constrain(oldAcc / ACC_MAX, -1.0f, 1.0f);
    float normJrk = constrain(oldJrk / JRK_MAX, -1.0f, 1.0f);
    float normCurr = constrain(oldCurr / CURR_MAX, -1.0f, 1.0f);
    float normSpeed = constrain(oldSpeed / SPEED_MAX, -1.0f, 1.0f);
    float normPErr = constrain(oldPErr / P_ERR_MAX, -1.0f, 1.0f);
    float normIErr = constrain(oldIErr / I_ERR_MAX, -1.0f, 1.0f);
    float normDErr = constrain(oldDErr / D_ERR_MAX, -1.0f, 1.0f);

    // Update weights with regularization to prevent explosion
    const float WEIGHT_DECAY = 0.999f; // Slight decay to prevent weights from growing indefinitely
    const float MAX_WEIGHT = 20.0f;    // Maximum weight magnitude

    weights[0] = weights[0] * WEIGHT_DECAY + ALPHA * tdError * 1.0f;
    weights[1] = constrain(weights[1] * WEIGHT_DECAY + ALPHA * tdError * normPos, -MAX_WEIGHT, MAX_WEIGHT);
    weights[2] = constrain(weights[2] * WEIGHT_DECAY + ALPHA * tdError * normSetpoint, -MAX_WEIGHT, MAX_WEIGHT);
    weights[3] = constrain(weights[3] * WEIGHT_DECAY + ALPHA * tdError * normVel, -MAX_WEIGHT, MAX_WEIGHT);
    weights[4] = constrain(weights[4] * WEIGHT_DECAY + ALPHA * tdError * normAcc, -MAX_WEIGHT, MAX_WEIGHT);
    weights[5] = constrain(weights[5] * WEIGHT_DECAY + ALPHA * tdError * normJrk, -MAX_WEIGHT, MAX_WEIGHT);
    weights[6] = constrain(weights[6] * WEIGHT_DECAY + ALPHA * tdError * normCurr, -MAX_WEIGHT, MAX_WEIGHT);
    weights[7] = constrain(weights[7] * WEIGHT_DECAY + ALPHA * tdError * normSpeed, -MAX_WEIGHT, MAX_WEIGHT);
    weights[8] = constrain(weights[8] * WEIGHT_DECAY + ALPHA * tdError * normPErr, -MAX_WEIGHT, MAX_WEIGHT);
    weights[9] = constrain(weights[9] * WEIGHT_DECAY + ALPHA * tdError * normIErr, -MAX_WEIGHT, MAX_WEIGHT);
    weights[10] = constrain(weights[10] * WEIGHT_DECAY + ALPHA * tdError * normDErr, -MAX_WEIGHT, MAX_WEIGHT);
}

float TinyML::getLastTDError()
{
    return lastTDError;
}

float TinyML::computeReward(float pos, float vel, float acc, float jrk, float curr)
{
    // Improved reward function for stable learning
    float positionError = abs(Motor::propError);
    float integralError = abs(Motor::intError);

    // Main objective: minimize position error (scaled appropriately)
    float positionReward = -positionError * 0.1f;

    // Penalize integral windup but less aggressively
    float integralPenalty = -integralError * 0.01f;

    // Small penalties for smooth motion (reduced scaling)
    float smoothnessPenalty = -abs(vel) * 0.005f - abs(acc) * 0.001f - abs(jrk) * 0.0001f;

    // Small penalty for high current (efficiency)
    float currentPenalty = -abs(curr) * 0.02f;

    // Combine rewards with reasonable scaling
    reward = positionReward + integralPenalty + smoothnessPenalty + currentPenalty;

    // Clip reward to reasonable range to prevent learning instability
    reward = constrain(reward, -10.0f, 10.0f);

    return reward;
}

void TinyML::TaskActionSelection(void *pvParameters)
{
    (void)pvParameters;
    int actionFrequencyHz = 5;  // Action selection frequency - reduced for stability
    TickType_t actionDelay = pdMS_TO_TICKS(1000 / actionFrequencyHz);

    // Store previous state for experience creation
    static float prevPos = 0.0f;
    static float prevSetpoint = 0.0f;
    static float prevVel = 0.0f;
    static float prevAcc = 0.0f;
    static float prevJrk = 0.0f;
    static float prevCurr = 0.0f;
    static float prevSpeed = 0.0f;
    static float prevPErr = 0.0f;
    static float prevIErr = 0.0f;
    static float prevDErr = 0.0f;
    static float prevAction = 0.0f;

    // Adaptive exploration based on TD error stability
    static float epsilon = EPSILON_MAX; // Start with high exploration

    for (;;)
    {
        TickType_t xLastWakeTime = xTaskGetTickCount();

        // Get current state - all 11 features
        float currPos = Motor::motorPos;
        float currSetpoint = Motor::motorSetpoint;
        float currVel = Motor::motorVel;
        float currAcc = Motor::motorAcc;
        float currJrk = Motor::motorJrk;
        float currCurr = Motor::motorCurrent;
        float currSpeed = Motor::motorSpeed / 255.0f; // Normalize to 0-1
        float currPErr = Motor::propError;
        float currIErr = Motor::intError;
        float currDErr = Motor::derivError;

        float chosenAction = computeAction(currPos, currSetpoint, currVel, currAcc, currJrk, currCurr, currSpeed, currPErr, currIErr, currDErr);

        // Adaptive exploration: scale epsilon based on TD error stability
        // High TD error = high epsilon (more exploration needed)
        // Low TD error = low epsilon (safe to exploit learned policy)
        float tdErrorScale = constrain(avgTDError / 2.0f, 0.0f, 1.0f); // Normalize to 0-1
        epsilon = EPSILON_BASE + (EPSILON_MAX - EPSILON_BASE) * tdErrorScale;

        // Add exploration with adaptive epsilon
        if (random(1000) < (epsilon * 1000)) {
            chosenAction = random(-200, 201); // Random action between -200 and 200 (safer range)
            Serial.print("EXPLORE: ");
            Serial.println(chosenAction);
            exploreActionCount++; // Track exploration actions
        }

        // Safety check: prevent extreme actions that could cause instability
        chosenAction = constrain(chosenAction, -180.0f, 180.0f); // Reduced range for safety

        // Add action smoothing to prevent oscillations (exponential moving average)
        static float smoothedAction = 0.0f;
        const float ACTION_SMOOTHING = 0.7f; // Higher = more smoothing
        smoothedAction = ACTION_SMOOTHING * smoothedAction + (1.0f - ACTION_SMOOTHING) * chosenAction;
        chosenAction = smoothedAction;

        Motor::motorSpeed = (int)chosenAction;

        if ((Motor::motorPos > POS_MAX && Motor::motorSpeed > 0) ||
            (Motor::motorPos < -POS_MAX && Motor::motorSpeed < 0))
        {
            Motor::motorSpeed = 0;
        }

        float reward = computeReward(Motor::motorPos, currVel, currAcc, currJrk, currCurr);

        // Send experience to learning task (if we have previous state)
        if (prevPos != 0.0f || prevSetpoint != 0.0f || prevPErr != 0.0f) {
            Experience exp;
            exp.oldPos = prevPos;
            exp.oldSetpoint = prevSetpoint;
            exp.oldVel = prevVel;
            exp.oldAcc = prevAcc;
            exp.oldJrk = prevJrk;
            exp.oldCurr = prevCurr;
            exp.oldSpeed = prevSpeed;
            exp.oldPErr = prevPErr;
            exp.oldIErr = prevIErr;
            exp.oldDErr = prevDErr;
            exp.action = prevAction;
            exp.reward = reward;
            exp.newPos = currPos;
            exp.newSetpoint = currSetpoint;
            exp.newVel = currVel;
            exp.newAcc = currAcc;
            exp.newJrk = currJrk;
            exp.newCurr = currCurr;
            exp.newSpeed = currSpeed;
            exp.newPErr = currPErr;
            exp.newIErr = currIErr;
            exp.newDErr = currDErr;
            
            // Non-blocking send - if queue is full, experience is dropped
            xQueueSend(experienceQueue, &exp, 0);
        }

        // Store current state as previous for next iteration
        prevPos = currPos;
        prevSetpoint = currSetpoint;
        prevVel = currVel;
        prevAcc = currAcc;
        prevJrk = currJrk;
        prevCurr = currCurr;
        prevSpeed = currSpeed;
        prevPErr = currPErr;
        prevIErr = currIErr;
        prevDErr = currDErr;
        prevAction = chosenAction;

        vTaskDelayUntil(&xLastWakeTime, actionDelay);
    }
}

void TinyML::TaskLearning(void *pvParameters)
{
    (void)pvParameters;
    int learningFrequencyHz = 1;  // Learning frequency - slower for stability
    TickType_t learningDelay = pdMS_TO_TICKS(1000 / learningFrequencyHz);

    // TD error tracking for adaptive exploration
    static float tdErrorSum = 0.0f;
    static int tdErrorCount = 0;

    // Performance tracking metrics
    static float trackingErrorSum = 0.0f;
    static float controlEffortSum = 0.0f;
    static int performanceSampleCount = 0;
    static int totalActionCount = 0;

    // Output header for metrics
    Serial.println("=== RL MOTOR CONTROL PERFORMANCE METRICS ===");
    Serial.println("Time(s),AvgTDError,Epsilon,MaxWeight,QueueSize,AvgTrackErr,AvgCtrlEffort,ExploreRate,Reward,LearningRate");

    for (;;)
    {
        TickType_t xLastWakeTime = xTaskGetTickCount();

        // Process all available experiences in the queue
        Experience exp;
        while (xQueueReceive(experienceQueue, &exp, 0) == pdTRUE) {
            // Update weights using this experience
            updateWeights(exp.oldPos, exp.oldSetpoint, exp.oldVel, exp.oldAcc, exp.oldJrk, exp.oldCurr, exp.oldSpeed, exp.oldPErr, exp.oldIErr, exp.oldDErr,
                         exp.action, exp.reward,
                         exp.newPos, exp.newSetpoint, exp.newVel, exp.newAcc, exp.newJrk, exp.newCurr, exp.newSpeed, exp.newPErr, exp.newIErr, exp.newDErr);

            // Track actual TD error for adaptive exploration
            tdErrorSum += getLastTDError();
            tdErrorCount++;

            // Track performance metrics
            trackingErrorSum += abs(exp.newPErr);  // Position tracking error
            controlEffortSum += abs(exp.action);    // Control effort (motor speed)
            performanceSampleCount++;

            // Track exploration vs exploitation
            totalActionCount++;
            // Note: exploration tracking is done in action selection task
        }

        // Update shared average TD error for adaptive exploration
        if (tdErrorCount > 0) {
            avgTDError = tdErrorSum / tdErrorCount;
        }

        // Print comprehensive performance metrics every 10 seconds
        static int debugCounter = 0;
        if (debugCounter++ % 10 == 0) { // Every 10 seconds
            // Calculate metrics
            float currentEpsilon = EPSILON_BASE + (EPSILON_MAX - EPSILON_BASE) * constrain(avgTDError / 2.0f, 0.0f, 1.0f);
            float maxWeight = 0;
            for (int i = 0; i < NUM_FEATURES; i++) {
                maxWeight = max(maxWeight, abs(weights[i]));
            }

            float avgTrackingError = (performanceSampleCount > 0) ? trackingErrorSum / performanceSampleCount : 0.0f;
            float avgControlEffort = (performanceSampleCount > 0) ? controlEffortSum / performanceSampleCount : 0.0f;
            float exploreRate = (totalActionCount > 0) ? (float)TinyML::exploreActionCount / totalActionCount : 0.0f;

            // Output in CSV format for easy parsing
            Serial.print(millis() / 1000.0f, 1);  // Time in seconds
            Serial.print(",");
            Serial.print(avgTDError, 4);         // Average TD error
            Serial.print(",");
            Serial.print(currentEpsilon, 3);     // Current exploration rate
            Serial.print(",");
            Serial.print(maxWeight, 3);          // Maximum weight magnitude
            Serial.print(",");
            Serial.print(uxQueueMessagesWaiting(experienceQueue)); // Queue size
            Serial.print(",");
            Serial.print(avgTrackingError, 3);   // Average tracking error
            Serial.print(",");
            Serial.print(avgControlEffort, 1);   // Average control effort
            Serial.print(",");
            Serial.print(exploreRate, 3);        // Exploration rate (0-1)
            Serial.print(",");
            Serial.print(reward, 2);             // Current reward
            Serial.print(",");
            Serial.println(ALPHA, 4);            // Learning rate

            // Reset performance tracking for next interval
            tdErrorSum = 0.0f;
            tdErrorCount = 0;
            trackingErrorSum = 0.0f;
            controlEffortSum = 0.0f;
            performanceSampleCount = 0;
            TinyML::exploreActionCount = 0;
            totalActionCount = 0;
        }

        vTaskDelayUntil(&xLastWakeTime, learningDelay);
    }
}