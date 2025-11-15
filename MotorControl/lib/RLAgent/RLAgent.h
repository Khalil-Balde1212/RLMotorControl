#ifndef SYSTEM_IDENTIFICATION_H
#define SYSTEM_IDENTIFICATION_H

#include <Arduino.h>
#include <ArduinoEigenDense.h>

using namespace Eigen;

struct SystemModel{
    const int NUMBER_OF_STATES = 4; // position, velocity, acceleration, jerk
    const int NUMBER_OF_CONTROL = 1; 

    MatrixXd stateTransition;
    MatrixXd responseVector;

    void initializeModel();
    MatrixXd predictNextState(const MatrixXd& currentState, const MatrixXd& input);

    void printMatrices() const;
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
#endif