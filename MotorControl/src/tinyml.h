#ifndef TINYML_H
#define TINYML_H

#define ALPHA 0.1f //learning rate
#define GAMMA 0.5f //discount factor
#define EPSILON 0.1f //exploration rate

// Discretization ranges (adjust based on your motor specs)
#define POS_MIN 0.0f       // Position min (radians, 0 = start)
#define POS_MAX 6.2832f    // Position max (2*pi radians, full rotation)
#define VEL_MIN -5.67f     // Velocity min (rad/s)
#define VEL_MAX 5.67f      // Velocity max (rad/s)
#define ACC_MIN -53.8f     // Acceleration min (rad/s^2)
#define ACC_MAX 53.8f      // Acceleration max (rad/s^2)
#define JRK_MIN -336.32f    // Jerk min (rad/s^3)
#define JRK_MAX 336.32f     // Jerk max (rad/s^3)
#define CURR_MIN 0.0f      // Current min (A)
#define CURR_MAX 2.0f      // Current max (A)

// Number of bins per variable (reduced for memory)
#define D_POS 3
#define D_VEL 3
#define D_ACC 3
#define D_JRK 3
#define D_CURR 3

#define NUM_STATES D_POS*D_VEL*D_ACC*D_JRK*D_CURR  // 3^5 = 243 states
#define NUM_ACTIONS 11 // Actions: motor speeds from -255 to 255 in steps

namespace TinyML
{
    extern float qTable[NUM_STATES][NUM_ACTIONS];
    extern int currentState;
    extern int action;
    extern float reward;

    void initializeQTable();
    int chooseAction(int state);

    // Discretization functions
    int discretizePos(float val);
    int discretizeVel(float val);
    int discretizeAcc(float val);
    int discretizeJrk(float val);
    int discretizeCurr(float val);

    // Get state index from discretized values
    int getStateIndex(int dPos, int dVel, int dAcc, int dJrk, int dCurr);
    void updateQTable(int oldState, int action, float reward, int newState);
    float computeReward();
    void TaskTinyML(void *pvParameters);
    void setup();
}

#endif