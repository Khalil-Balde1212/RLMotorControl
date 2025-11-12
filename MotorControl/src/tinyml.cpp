#include "tinyml.h"
#include <Arduino.h>
#include <FreeRTOS_SAMD21.h>
#include "motorControl.h"

// Define extern variables
float TinyML::qTable[NUM_STATES][NUM_ACTIONS];
int TinyML::currentState = 0;
int TinyML::action = 0;
float TinyML::reward = 0.0f;

void TinyML::setup()
{
    initializeQTable();

    xTaskCreate(TinyML::TaskTinyML, "TinyMLTask", 512, NULL, 1, NULL);
}

void TinyML::initializeQTable()
{
    for (int i = 0; i < NUM_STATES; ++i)
    {
        for (int j = 0; j < NUM_ACTIONS; ++j)
        {
            qTable[i][j] = 0.0f; // Initialize all Q-values to zero
        }
    }
}

int TinyML::chooseAction(int state)
{
    if (random(0, 100) < EPSILON * 100)
    {
        // Explore: choose a random action
        action = random(0, NUM_ACTIONS);
    }
    else
    {
        // Exploit: choose the best known action
        float maxQ = qTable[state][0];
        action = 0;
        for (int j = 1; j < NUM_ACTIONS; ++j)
        {
            if (qTable[state][j] > maxQ)
            {
                maxQ = qTable[state][j];
                action = j;
            }
        }
    }
    return action;
}


void TinyML::updateQTable(int state, int action, float reward, int nextState) {
  float oldQValue = qTable[state][action];
  float maxNextQValue = qTable[nextState][0];
  for (int i = 1; i < NUM_ACTIONS; i++) {
    if (qTable[nextState][i] > maxNextQValue) {
      maxNextQValue = qTable[nextState][i];
    }
  }
  qTable[state][action] = oldQValue + ALPHA * (reward + GAMMA * maxNextQValue - oldQValue);
}


void TinyML::TaskTinyML(void *pvParameters)
{
    (void)pvParameters;
    int taskFrequencyHz = 10;
    TickType_t delay = pdMS_TO_TICKS(1000 / taskFrequencyHz);

    for (;;)
    {
        TickType_t xLastWakeTime = xTaskGetTickCount();

        // Discretize current motor state
        int dPos = discretizePos(Motor::motorPos);
        int dVel = discretizeVel(Motor::motorVel);
        int dAcc = discretizeAcc(Motor::motorAcc);
        int dJrk = discretizeJrk(Motor::motorJrk);
        int dCurr = discretizeCurr(Motor::motorCurrent);

        int newState = getStateIndex(dPos, dVel, dAcc, dJrk, dCurr);

        // Choose action based on current state
        int chosenAction = chooseAction(currentState);

        // Execute action: here we set motor speed based on action
        Motor::motorSpeed = (chosenAction - 5) * 51;
        Motor::motorSpeed = constrain(Motor::motorSpeed, -255, 255);

        // Update Q-table
        updateQTable(currentState, chosenAction, computeReward(), newState);

        currentState = newState;

        vTaskDelayUntil(&xLastWakeTime, delay);
    }
}

float TinyML::computeReward()
{
    float reward = 0.0f;
    reward -= Motor::motorJrk * 0.01f; // Penalize high jerk
    reward -= abs(Motor::motorPos - Motor::motorSetpoint) * 0.05f; // Penalize distance from target position (pi radians)

    reward -= abs(Motor::motorPos - POS_MAX) * 0.05f; // Penalize for getting close to max distance


    // Reward for stopping near setpoint
    if (Motor::motorVel == 0.0f && abs(Motor::motorPos - Motor::motorSetpoint) < 0.05f) {
        reward += 5.0f;
    }
    return reward;
}