#include "RLAgent.h"
#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <vector>
using namespace Eigen;
namespace RLPolicy
{
	const int HORIZON_SIZE = 5;
	const int STATE_SIZE = 4;
	const int HIDDEN_SIZE = 16;

	// LTI matrices using Eigen
	static Eigen::Matrix<float, STATE_SIZE, STATE_SIZE> stateTransitionMatrix =
		(Eigen::Matrix<float, STATE_SIZE, STATE_SIZE>() << 1.0f, 0.01f, 0.0f, 0.0f,
		 0.0f, 1.0f, 0.01f, 0.0f,
		 0.0f, 0.0f, 1.0f, 0.01f,
		 0.0f, 0.0f, 0.0f, 1.0f)
			.finished();

	static Eigen::Matrix<float, STATE_SIZE, 1> B =
		(Eigen::Matrix<float, STATE_SIZE, 1>() << 0.0f, 0.01f, 0.001f, 0.0001f).finished();

	// NN weights
	static Matrix<float, HIDDEN_SIZE, STATE_SIZE * 2 + 2> W1; // add explicit setpoint input
	static Matrix<float, HIDDEN_SIZE, 1> b1;
	static Matrix<float, Eigen::Dynamic, HIDDEN_SIZE> W2;
	static Matrix<float, Eigen::Dynamic, 1> b2;

	// Cached forward pass values for learning
	static std::vector<Matrix<float, STATE_SIZE * 2 + 2, 1>> cachedStates; // state, state_t+1, u_prev, setpoint vector
	static std::vector<Matrix<float, HIDDEN_SIZE, 1>> cachedHiddenLayer;   // biggums
	static std::vector<float> cachedActions;
	static float cachedSetpoint = 0.0f;
	static std::vector<float> cachedPos;

	static inline float ReLU(float x) { return x > 0.0f ? x : 0.0f; }

	void initializePolicy()
	{
		// stateTransitionMatrix stays as default unless user sets it elsewhere

		// Initialize weights with small random values
		W1 = Matrix<float, HIDDEN_SIZE, STATE_SIZE * 2 + 2>::Random() * 0.1f;
		b1 = Matrix<float, HIDDEN_SIZE, 1>::Zero();
		W2 = Matrix<float, Eigen::Dynamic, HIDDEN_SIZE>::Random(HORIZON_SIZE, HIDDEN_SIZE) * 0.1f;
		b2 = Matrix<float, Eigen::Dynamic, 1>::Zero(HORIZON_SIZE, 1);
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
		cachedStates.clear();
		cachedHiddenLayer.clear();
		cachedActions.clear();
		cachedPos.clear();
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
			cachedStates.push_back(x);
			cachedHiddenLayer.push_back(y);
			cachedActions.push_back(u_out_norm);
			// compute next normalized state using model / LTI approximation
			Matrix<float, STATE_SIZE, 1> xf;
			for (int s = 0; s < STATE_SIZE; ++s)
				xf(s, 0) = static_cast<float>(currentStateNormalized(s, 0));
			Matrix<float, STATE_SIZE, 1> nextState = stateTransitionMatrix * stateF + B * u_out_norm;
			cachedPos.push_back(nextState(0, 0));
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
		if (cachedPos.empty())
			return 0.0f;

		float reward = 0.0f;
		// weights
		float w_pos = 1.0f;
		float w_u = 0.01f;
		float w_du = 0.05f;
		float prevU = cachedActions.front();
		for (size_t i = 0; i < cachedPos.size(); ++i)
		{
			float pos = cachedPos[i];
			// use cached setpoint (normalized)
			float setpoint = cachedSetpoint;
			float err = setpoint - pos;
			reward += -w_pos * err * err;
			float u = cachedActions[i];
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
		if (cachedPos.empty())
			return;

		float policyLR = 1e-3f;
		// regularization
		float l2Reg = 1e-4f;

		// simple cost weights, must match calculateReward structure
		float w_pos = 1.0f;
		float w_u = 0.01f;
		float w_du = 0.05f;

		// Gradients init
		Matrix<float, Eigen::Dynamic, HIDDEN_SIZE> dW2 = Matrix<float, Eigen::Dynamic, HIDDEN_SIZE>::Zero(HORIZON_SIZE, HIDDEN_SIZE);
		Matrix<float, Eigen::Dynamic, 1> db2 = Matrix<float, Eigen::Dynamic, 1>::Zero(HORIZON_SIZE, 1);
		Matrix<float, HIDDEN_SIZE, STATE_SIZE * 2 + 2> dW1 = Matrix<float, HIDDEN_SIZE, STATE_SIZE * 2 + 2>::Zero();
		Matrix<float, HIDDEN_SIZE, 1> db1 = Matrix<float, HIDDEN_SIZE, 1>::Zero();

		float prevU = cachedActions.front();
		for (int t = 0; t < (int)cachedPos.size(); ++t)
		{
			float pos = cachedPos[t];
			float setpoint = cachedSetpoint;
			// derivative of loss w.r.t position
			float dL_dpos = 2.0f * w_pos * (pos - setpoint);
			// approximate dpos / doutput = A(0,1) * B(1,0) (control -> velocity -> position)
			float dpos_dout = stateTransitionMatrix(0, 1) * B(1, 0);
			// derivative w.r.t output (residual)
			float dL_dout = dL_dpos * dpos_dout + 2.0f * w_u * cachedActions[t] + 2.0f * w_du * (cachedActions[t] - prevU);

			// dW2 gradient (row t) = dL_dout * y^T
			for (int j = 0; j < HIDDEN_SIZE; ++j)
			{
				dW2(t, j) += dL_dout * cachedHiddenLayer[t](j, 0);
			}
			db2(t, 0) += dL_dout;

			// Backprop to W1: d output / d W1 = W2_row * diag(relu') * x^T
			Matrix<float, HIDDEN_SIZE, 1> w2row = W2.row(t).transpose();
			for (int j = 0; j < HIDDEN_SIZE; ++j)
			{
				float reluDeriv = cachedHiddenLayer[t](j, 0) > 0.0f ? 1.0f : 0.0f;
				float coeff = dL_dout * w2row(j, 0) * reluDeriv;
				for (int k = 0; k < STATE_SIZE * 2 + 2; ++k)
				{
					dW1(j, k) += coeff * cachedStates[t](k, 0);
				}
				db1(j, 0) += coeff;
			}
			prevU = cachedActions[t];
		}

		// Apply gradients with learning rate and tiny regularization
		W2 -= policyLR * (dW2 + l2Reg * W2);
		b2 -= policyLR * (db2 + l2Reg * b2);
		W1 -= policyLR * (dW1 + l2Reg * W1);
		b1 -= policyLR * (db1 + l2Reg * b1);
	}
}