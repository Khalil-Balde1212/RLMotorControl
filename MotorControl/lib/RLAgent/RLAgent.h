#ifndef SYSTEM_IDENTIFICATION_H
#define SYSTEM_IDENTIFICATION_H

#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <vector>

using namespace Eigen;

struct SystemModel{
    const int NUMBER_OF_STATES = 4; // position, velocity, acceleration, jerk
    const int NUMBER_OF_CONTROL = 1; 

    MatrixXd stateTransition;
    MatrixXd responseVector;

    // State vector layout (NUMBER_OF_STATES = 4):
    //   state[0] = position (radians) normalized to [-1,1] by MAX_POSITION
    //   state[1] = velocity (rad/s) normalized to [-1,1] by MAX_VELOCITY
    //   state[2] = acceleration (rad/s^2) normalized to [-1,1] by MAX_ACCELERATION
    //   state[3] = jerk (rad/s^3) normalized to [-1,1] by MAX_JERK

    void initializeModel();
    MatrixXd predictNextState(const MatrixXd& currentState, const MatrixXd& input);

    void printMatrices() const;
    String getMatricesCSV() const;
};

namespace SystemIdentification {
    extern float learningRate;
    extern SystemModel model;
    extern MatrixXd lastState;
    extern bool converged;
    extern float currentUpdateNorm;
    extern float currentErrorNorm;

    void initialize();
    void update(MatrixXd currentState, MatrixXd input);
}

namespace RLPolicy {
    extern const int HIDDEN_SIZE;
    extern const int HORIZON_SIZE;

    void initializePolicy();
    // Compute a sequence of motor commands (int in [-255,255]) for HORIZON_SIZE future steps
    std::vector<int> computeMotorSequence(const MatrixXd &currentStateNormalized, float u_prev_normalized, float setpoint_normalized);
    float calculateReward();
    void updatePolicyWeights();
}


#endif