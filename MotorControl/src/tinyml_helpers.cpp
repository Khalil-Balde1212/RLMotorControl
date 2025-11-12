#include "tinyml.h"
#include <Arduino.h>

// Discretization functions
int TinyML::discretizePos(float val) {
    val = constrain(val, POS_MIN, POS_MAX);
    return (int)((val - POS_MIN) / (POS_MAX - POS_MIN) * (D_POS - 1));
}

int TinyML::discretizeVel(float val) {
    val = constrain(val, VEL_MIN, VEL_MAX);
    return (int)((val - VEL_MIN) / (VEL_MAX - VEL_MIN) * (D_VEL - 1));
}

int TinyML::discretizeAcc(float val) {
    val = constrain(val, ACC_MIN, ACC_MAX);
    return (int)((val - ACC_MIN) / (ACC_MAX - ACC_MIN) * (D_ACC - 1));
}

int TinyML::discretizeJrk(float val) {
    val = constrain(val, JRK_MIN, JRK_MAX);
    return (int)((val - JRK_MIN) / (JRK_MAX - JRK_MIN) * (D_JRK - 1));
}

int TinyML::discretizeCurr(float val) {
    val = constrain(val, CURR_MIN, CURR_MAX);
    return (int)((val - CURR_MIN) / (CURR_MAX - CURR_MIN) * (D_CURR - 1));
}

int TinyML::getStateIndex(int dPos, int dVel, int dAcc, int dJrk, int dCurr) {
    return dPos + 
           dVel * D_POS + 
           dAcc * D_POS * D_VEL + 
           dJrk * D_POS * D_VEL * D_ACC + 
           dCurr * D_POS * D_VEL * D_ACC * D_JRK;
}