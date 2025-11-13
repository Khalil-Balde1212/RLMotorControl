#ifndef TINYML_H
#define TINYML_H

#define ALPHA 0.01f //learning rate
#define GAMMA 0.9f //discount factor
#define EPSILON 0.2f //exploration rate

// Normalization constants (max absolute values for features)
#define POS_MAX 6.283185f  // 2*pi radians
#define VEL_MAX 25.0f      // rad/s
#define ACC_MAX 100.0f      // rad/s^2
#define JRK_MAX 300.0f     // rad/s^3
#define CURR_MAX 2.0f      // A
#define ERR_MAX 50.0f      // accumulated position error

// State features: 1 + pos + vel + acc + jrk + curr + err = 7 total (action is output, not input)
#define NUM_FEATURES 7

namespace TinyML
{
    extern float weights[NUM_FEATURES]; // Linear weights for Q(s,a)
    extern int action;
    extern float reward;

    void initializeWeights();
    float computeAction(float pos, float vel, float acc, float jrk, float curr, float accumErr);

    // Compute value function for given state (no action needed since action is output)
    float computeValue(float pos, float vel, float acc, float jrk, float curr, float accumErr);

    // Update weights based on TD error for continuous actions
    void updateWeights(float oldPos, float oldVel, float oldAcc, float oldJrk, float oldCurr, float oldAccumErr, float action, float reward, float newPos, float newVel, float newAcc, float newJrk, float newCurr, float newAccumErr);

    float computeReward(float pos, float vel, float acc, float jrk, float curr);
    void TaskTinyML(void *pvParameters);
    void setup();
}

#endif