#include "RLAgent.h"

#include <Arduino.h>
#include <ArduinoEigenDense.h>

using namespace Eigen;
void SystemModel::initializeModel() {
    stateTransition = MatrixXd::Identity(NUMBER_OF_STATES, NUMBER_OF_STATES);
    responseVector = MatrixXd::Zero(NUMBER_OF_STATES, NUMBER_OF_CONTROL);
}

MatrixXd SystemModel::predictNextState(const MatrixXd& currentState, const MatrixXd& input) {
    return stateTransition * currentState + responseVector * input;
}


void SystemModel::printMatrices() const {
    Serial.println("State Transition Matrix:");
    for (int i = 0; i < stateTransition.rows(); ++i) {
        for (int j = 0; j < stateTransition.cols(); ++j) {
            Serial.print(stateTransition(i, j), 6);
            Serial.print("\t");
        }
        Serial.println();
    }
    Serial.println("Response Vector:");
    for (int i = 0; i < responseVector.rows(); ++i) {
        for (int j = 0; j < responseVector.cols(); ++j) {
            Serial.print(responseVector(i, j), 6);
            Serial.print("\t");
        }
        Serial.println();
    }
}


namespace SystemIdentification {
    SystemModel model = SystemModel();
    MatrixXd lastState;
    float learningRate = 0.0001f;
    bool converged = false;
    float currentUpdateNorm = 0.0f;
    float currentErrorNorm = 0.0f;

    void initialize() {
        model.initializeModel();
        lastState = MatrixXd::Zero(model.NUMBER_OF_STATES, 1);
    }

    void update(MatrixXd currentState, MatrixXd input) {
        // predict next state
        MatrixXd predictedState = model.predictNextState(lastState, input);

        // compute error
        MatrixXd error = currentState - predictedState;
        currentErrorNorm = error.norm();
        
        //update StateTransition and ResponseVector matrices based on error
        MatrixXd updateST = learningRate * (error * lastState.transpose());
        MatrixXd updateRV = learningRate * (error * input.transpose());
        model.stateTransition = model.stateTransition + updateST;
        model.responseVector = model.responseVector + updateRV;
        currentUpdateNorm = updateST.norm() + updateRV.norm(); // Combined update norm as convergence rate

        // Check for convergence
        static int convergenceCounter = 0;
        if (error.norm() < 0.01) {
            convergenceCounter++;
            if (convergenceCounter > 100) {
                converged = true;
            }
        } else {
            convergenceCounter = 0;
        }

        lastState = currentState;
    }
}
