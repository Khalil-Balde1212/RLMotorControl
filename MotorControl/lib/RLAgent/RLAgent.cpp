#include "RLAgent.h"

#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <vector>

using namespace Eigen;
void SystemModel::initializeModel()
{
    stateTransition = MatrixXd::Ones(NUMBER_OF_STATES, NUMBER_OF_STATES);
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
        float accuracyThreshold = 5e-2; // Adjust as needed
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
        float stagnationThreshold = 7e-2; // Adjust as needed
        static float prevErrorNorm = 0.0f;
        if (fabs(prevErrorNorm - currentErrorNorm) < stagnationThreshold)
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

namespace RLPolicy
{
    const int HORIZON_SIZE = 5;
    const int STATE_SIZE = 4;
    const int HIDDEN_SIZE = 16;

    // LTI matrices using Eigen
    static Eigen::Matrix<float, STATE_SIZE, STATE_SIZE> A = (Eigen::Matrix<float, STATE_SIZE, STATE_SIZE>() << 1.0f, 0.01f, 0.0f, 0.0f,
                                                             0.0f, 1.0f, 0.01f, 0.0f,
                                                             0.0f, 0.0f, 1.0f, 0.01f,
                                                             0.0f, 0.0f, 0.0f, 1.0f)
                                                                .finished();

    static Eigen::Matrix<float, STATE_SIZE, 1> B = (Eigen::Matrix<float, STATE_SIZE, 1>() << 0.0f, 0.01f, 0.001f, 0.0001f).finished();

    // NN weights
    static Eigen::Matrix<float, HIDDEN_SIZE, STATE_SIZE * 2 + 2> W1; // add explicit setpoint input
    static Eigen::Matrix<float, HIDDEN_SIZE, 1> b1;
    static Eigen::Matrix<float, Eigen::Dynamic, HIDDEN_SIZE> W2; // HORIZON_SIZE x HIDDEN_SIZE
    static Eigen::Matrix<float, Eigen::Dynamic, 1> b2;           // HORIZON_SIZE x 1

    // Cached forward pass values for learning
    static std::vector<Eigen::Matrix<float, STATE_SIZE * 2 + 2, 1>> cachedX;
    static std::vector<Eigen::Matrix<float, HIDDEN_SIZE, 1>> cachedY;
    static std::vector<float> cachedU;
    static std::vector<float> cachedVel;
    static float cachedSetpoint = 0.0f;

    static inline float ReLU(float x) { return x > 0.0f ? x : 0.0f; }

    void initializePolicy()
    {
        // Initialize weights with small random values
        W1 = Eigen::Matrix<float, HIDDEN_SIZE, STATE_SIZE * 2 + 2>::Random() * 0.1f;
        b1 = Eigen::Matrix<float, HIDDEN_SIZE, 1>::Zero();
        W2 = Eigen::Matrix<float, Eigen::Dynamic, HIDDEN_SIZE>::Random(HORIZON_SIZE, HIDDEN_SIZE) * 0.1f;
        b2 = Eigen::Matrix<float, Eigen::Dynamic, 1>::Zero(HORIZON_SIZE, 1);
        // clear cached last outputs
        // (use vectors sized to HORIZON_SIZE)
        // lastXList/lastYList/lastUList are static and will be cleared next time computeMotorSequence runs
    }

    std::vector<int> computeMotorSequence(const MatrixXd &currentStateNormalized, float u_prev_normalized, float setpoint_normalized)
    {
        // Prepare input vector x
        Eigen::Matrix<float, STATE_SIZE * 2 + 2, 1> x;
        for (int i = 0; i < STATE_SIZE; ++i)
        {
            x(i, 0) = static_cast<float>(currentStateNormalized(i, 0));
            x(i + STATE_SIZE, 0) = static_cast<float>(currentStateNormalized(i, 0) - setpoint_normalized);
        }
        x(STATE_SIZE * 2, 0) = u_prev_normalized;
        // Add explicit setpoint input as extra feature
        x(STATE_SIZE * 2 + 1, 0) = setpoint_normalized;

        // Forward pass through the NN
        Eigen::Matrix<float, HIDDEN_SIZE, 1> y = W1 * x + b1;
        for (int i = 0; i < HIDDEN_SIZE; ++i)
        {
            y(i, 0) = ReLU(y(i, 0));
        }

        // Compute HORIZON outputs (residuals)
        Eigen::Matrix<float, Eigen::Dynamic, 1> output = W2 * y + b2; // HORIZON_SIZE vector
        std::vector<int> cmds;
        cmds.reserve(HORIZON_SIZE);
        // store last computations for reward and learning
        cachedX.clear();
        cachedY.clear();
        cachedU.clear();
        cachedVel.clear();
        cachedSetpoint = setpoint_normalized;
        // simulate future states using LTI A,B and the chosen controls
        Eigen::Matrix<float, STATE_SIZE, 1> stateF;
        for (int s = 0; s < STATE_SIZE; ++s)
            stateF(s, 0) = static_cast<float>(currentStateNormalized(s, 0));
        for (int i = 0; i < HORIZON_SIZE; ++i)
        {
            float u_out_norm = u_prev_normalized + output(i, 0);
            // clamp to [-1,1]
            if (u_out_norm > 1.0f)
                u_out_norm = 1.0f;
            if (u_out_norm < -1.0f)
                u_out_norm = -1.0f;
            int motorCmd = (int)round(u_out_norm * 255.0f);
            if (motorCmd > 255)
                motorCmd = 255;
            if (motorCmd < -255)
                motorCmd = -255;
            cmds.push_back(motorCmd);
            // cache for reward/learning
            cachedX.push_back(x);
            cachedY.push_back(y);
            cachedU.push_back(u_out_norm);
            // compute next normalized state using model / LTI approximation
            Eigen::Matrix<float, STATE_SIZE, 1> xf;
            for (int s = 0; s < STATE_SIZE; ++s)
                xf(s, 0) = static_cast<float>(currentStateNormalized(s, 0));
            Eigen::Matrix<float, STATE_SIZE, 1> nextState = A * stateF + B * u_out_norm;
            cachedVel.push_back(nextState(1, 0));
            // update current state for next horizon step (assume we take this action)
            stateF = nextState;
        }
        // caches already updated

        return cmds;
    }

    // Compute the reward for the last computed trajectory stored in cached lists
    float calculateReward()
    {
        using namespace SystemIdentification;
        // Access cached arrays in this namespace
        if (cachedVel.empty())
            return 0.0f;

        float reward = 0.0f;
        // weights
        float w_vel = 1.0f;
        float w_u = 0.01f;
        float w_du = 0.05f;
        float prevU = cachedU.front();
        for (size_t i = 0; i < cachedVel.size(); ++i)
        {
            float vel = cachedVel[i];
            // use cached setpoint (normalized)
            float setpoint = cachedSetpoint;
            float err = setpoint - vel;
            reward += -w_vel * err * err;
            float u = cachedU[i];
            reward += -w_u * u * u;
            float du = u - prevU;
            reward += -w_du * du * du;
            prevU = u;
        }
        return reward;
    }

    void updatePolicyWeights()
    {
        // Lightweight gradient update using cached forward pass
        // Access cached arrays in this namespace
        if (cachedVel.empty())
            return;

        float policyLR = 1e-3f;
        // regularization
        float l2Reg = 1e-4f;

        // simple cost weights, must match calculateReward structure
        float w_vel = 1.0f;
        float w_u = 0.01f;
        float w_du = 0.05f;

        // Gradients init
        Eigen::Matrix<float, Eigen::Dynamic, HIDDEN_SIZE> dW2 = Eigen::Matrix<float, Eigen::Dynamic, HIDDEN_SIZE>::Zero(HORIZON_SIZE, HIDDEN_SIZE);
        Eigen::Matrix<float, Eigen::Dynamic, 1> db2 = Eigen::Matrix<float, Eigen::Dynamic, 1>::Zero(HORIZON_SIZE, 1);
        Eigen::Matrix<float, HIDDEN_SIZE, STATE_SIZE * 2 + 2> dW1 = Eigen::Matrix<float, HIDDEN_SIZE, STATE_SIZE * 2 + 2>::Zero();
        Eigen::Matrix<float, HIDDEN_SIZE, 1> db1 = Eigen::Matrix<float, HIDDEN_SIZE, 1>::Zero();

        float prevU = cachedU.front();
        for (int t = 0; t < (int)cachedVel.size(); ++t)
        {
            float vel = cachedVel[t];
            float setpoint = cachedSetpoint;
            // derivative of loss w.r.t vel
            float dL_dvel = 2.0f * w_vel * (vel - setpoint);
            // approximate dvel / doutput = B[1] (effect of control on velocity through LTI)
            float dvel_dout = B(1, 0);
            // derivative w.r.t output (residual)
            float dL_dout = dL_dvel * dvel_dout + 2.0f * w_u * cachedU[t] + 2.0f * w_du * (cachedU[t] - prevU);

            // dW2 gradient (row t) = dL_dout * y^T
            for (int j = 0; j < HIDDEN_SIZE; ++j)
            {
                dW2(t, j) += dL_dout * cachedY[t](j, 0);
            }
            db2(t, 0) += dL_dout;

            // Backprop to W1: d output / d W1 = W2_row * diag(relu') * x^T
            Eigen::Matrix<float, HIDDEN_SIZE, 1> w2row = W2.row(t).transpose();
            for (int j = 0; j < HIDDEN_SIZE; ++j)
            {
                float reluDeriv = cachedY[t](j, 0) > 0.0f ? 1.0f : 0.0f;
                float coeff = dL_dout * w2row(j, 0) * reluDeriv;
                for (int k = 0; k < STATE_SIZE * 2 + 2; ++k)
                {
                    dW1(j, k) += coeff * cachedX[t](k, 0);
                }
                db1(j, 0) += coeff;
            }
            prevU = cachedU[t];
        }

        // Apply gradients with learning rate and tiny regularization
        W2 -= policyLR * (dW2 + l2Reg * W2);
        b2 -= policyLR * (db2 + l2Reg * b2);
        W1 -= policyLR * (dW1 + l2Reg * W1);
        b1 -= policyLR * (db1 + l2Reg * b1);
    }
}