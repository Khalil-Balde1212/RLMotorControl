#ifndef TINYML_H
#define TINYML_H

#include <FreeRTOS_SAMD21.h>  // Include FreeRTOS for QueueHandle_t

#define ALPHA 0.2f           // Conservative learning rate
#define GAMMA 0.95f           // Higher discount for future rewards
#define EPSILON_BASE 0.05f    // Base exploration rate (minimum)
#define EPSILON_MAX 0.8f      // Maximum exploration rate
#define TD_ERROR_WINDOW 50    // Number of TD errors to average for sstability

// Normalization constants (max absolute values for features) - Updated based on observed data
// These ranges ensure all features are normalized to [-1, 1] for Q-learning
#define POS_MAX 10.0f      // Position in radians (observed: ~8.17)
#define SETPOINT_MAX 10.0f // Setpoint in radians (observed: ~6.28)
#define VEL_MAX 60.0f      // Velocity in rad/s (observed: ~52.25)
#define ACC_MAX 2500.0f    // Acceleration in rad/s² (observed: ~2236)
#define JRK_MAX 600000.0f  // Jerk in rad/s³ (observed: ~504876)
#define CURR_MAX 5.0f      // Current in A (max possible: 5A sensor)
#define SPEED_MAX 1.0f     // Motor speed as percentage (0-1)
#define P_ERR_MAX 20.0f    // P error (observed: ~13.62)
#define I_ERR_MAX 50.0f    // I error (observed: ~7.49, constrained to 50)
#define D_ERR_MAX 60.0f    // D error (observed: ~51.81)
#define REWARD_MAX 10.0f   // Reward range (estimated)

// State features: bias + pos + setpoint + vel + acc + jrk + curr + speed + p_err + i_err + d_err = 12 total
#define NUM_FEATURES 12

// Experience buffer for learning task
struct Experience {
    float oldPos, oldSetpoint, oldVel, oldAcc, oldJrk, oldCurr, oldSpeed, oldPErr, oldIErr, oldDErr;
    float action, reward;
    float newPos, newSetpoint, newVel, newAcc, newJrk, newCurr, newSpeed, newPErr, newIErr, newDErr;
};

namespace TinyML
{
    extern float weights[NUM_FEATURES]; // Linear weights for Q(s,a)
    extern int action;
    extern float reward;
    extern float avgTDError; // Shared TD error average for adaptive exploration
    extern int exploreActionCount; // Track exploration actions for metrics

    void initializeWeights();
    float computeAction(float pos, float setpoint, float vel, float acc, float jrk, float curr, float speed, float pErr, float iErr, float dErr);

    // Compute value function for given state (no action needed since action is output)
    float computeValue(float pos, float setpoint, float vel, float acc, float jrk, float curr, float speed, float pErr, float iErr, float dErr);

    // Update weights based on TD error for continuous actions
    void updateWeights(float oldPos, float oldSetpoint, float oldVel, float oldAcc, float oldJrk, float oldCurr, float oldSpeed, float oldPErr, float oldIErr, float oldDErr, float action, float reward, float newPos, float newSetpoint, float newVel, float newAcc, float newJrk, float newCurr, float newSpeed, float newPErr, float newIErr, float newDErr);

    // Get the last computed TD error for adaptive exploration
    float getLastTDError();

    float computeReward(float pos, float vel, float acc, float jrk, float curr);
    void TaskActionSelection(void *pvParameters);
    void TaskLearning(void *pvParameters);
    void setup();
}

// Declare queue handle outside namespace
extern QueueHandle_t experienceQueue;

#endif