#include "RLAgent.h"

#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <vector>

using namespace Eigen;
void SystemModel::initializeModel()
{
    stateTransition = MatrixXd::Identity(NUMBER_OF_STATES, NUMBER_OF_STATES);
    responseVector = MatrixXd::Ones(NUMBER_OF_STATES, NUMBER_OF_CONTROL);
}

MatrixXd SystemModel::predictNextState(const MatrixXd &currentState, const MatrixXd &input)
{
    return stateTransition * currentState + responseVector * input;
}

void SystemModel::printMatrices() const
{
    Serial.println("State Transition Matrix:");
    for (int i = 0; i < stateTransition.rows(); ++i)
    {
        for (int j = 0; j < stateTransition.cols(); ++j)
        {
            Serial.print(stateTransition(i, j), 6);
            Serial.print("\t");
        }
        Serial.println();
    }
    Serial.println("Response Vector:");
    for (int i = 0; i < responseVector.rows(); ++i)
    {
        for (int j = 0; j < responseVector.cols(); ++j)
        {
            Serial.print(responseVector(i, j), 6);
            Serial.print("\t");
        }
        Serial.println();
    }
}

String SystemModel::getMatricesCSV() const
{
    String csv = "";
    // State Transition Matrix (4x4)
    for (int i = 0; i < stateTransition.rows(); ++i)
    {
        for (int j = 0; j < stateTransition.cols(); ++j)
        {
            csv += String(stateTransition(i, j), 6) + ",";
        }
    }
    // Response Vector (4x1)
    for (int i = 0; i < responseVector.rows(); ++i)
    {
        for (int j = 0; j < responseVector.cols(); ++j)
        {
            csv += String(responseVector(i, j), 6) + ",";
        }
    }
    return csv;
}

namespace SystemIdentification
{
    SystemModel model = SystemModel();
    MatrixXd lastState;
    float learningRate = 0.5f;
    bool converged = false;
    float currentUpdateNorm = 0.0f;
    float currentErrorNorm = 0.0f;

    void initialize()
    {
        model.initializeModel();
        lastState = MatrixXd::Zero(model.NUMBER_OF_STATES, 1);
    }

    void update(MatrixXd currentState, MatrixXd input)
    {
        // predict next state
        MatrixXd predictedState = model.predictNextState(lastState, input);

        // compute error
        MatrixXd error = currentState - predictedState;
        currentErrorNorm = error.norm();

        // update StateTransition and ResponseVector matrices based on error
        MatrixXd updateST = learningRate * (error * lastState.transpose());
        MatrixXd updateRV = learningRate * (error * input.transpose());
        model.stateTransition = model.stateTransition + updateST;
        model.responseVector = model.responseVector + updateRV;
        currentUpdateNorm = updateST.norm() + updateRV.norm(); // Combined update norm as convergence rate

        // Check for convergence
        static int accuracyCounter = 0;
        static bool accuracyConverged = false;
        float accuracyThreshold = 5e-1; // Adjust as needed

        // Apply EMA filter to currentErrorNorm
        static float emaErrorNorm = 0.0f;
        float emaAlpha = 0.1f; // Smoothing factor, adjust as needed
        emaErrorNorm = emaAlpha * currentErrorNorm + (1.0f - emaAlpha) * emaErrorNorm;
        if (fabs(currentErrorNorm) < accuracyThreshold)
        {
            if (++accuracyCounter >= 10) // Require 10 consecutive accurate iterations
            {
                accuracyConverged = true;
            }
        }
        else
        {
            accuracyCounter = 0;
        }

        static int stagnationCounter = 0;
        static bool stagnationConverged = false;
        float stagnationThreshold = 5e-1; // Adjust as needed
        static float prevErrorNorm = 0.0f;
        // Apply EMA filter to error norm difference for stagnation detection
        static float emaErrorDiff = 0.0f;
        float emaDiffAlpha = 0.1f; // Smoothing factor, adjust as needed
        float errorDiff = fabs(prevErrorNorm - currentErrorNorm);
        emaErrorDiff = emaDiffAlpha * errorDiff + (1.0f - emaDiffAlpha) * emaErrorDiff;

        if (emaErrorDiff < stagnationThreshold)
        {
            stagnationCounter++;
            if (stagnationCounter > 25)
            {
            stagnationConverged = true;
            }
        }
        else
        {
            stagnationCounter = 0;
        }

        converged = accuracyConverged && stagnationConverged;
        prevErrorNorm = currentErrorNorm;

        lastState = currentState;
    }
}

