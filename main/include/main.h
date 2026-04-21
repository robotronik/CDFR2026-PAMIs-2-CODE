#pragma once 
#include "locomotion/motor_control.h"
#include "actuators/pull_switch.h"
#include "actuators/leds.h"

// Pami 0 is ninja, so we can disable all the code related to it for now. 
// This will make it easier to test the rest of the code
#ifndef N_PAMI
#define N_PAMI 0
#endif

#if N_PAMI == 0
#define NINJA
#endif

enum MainFSM_State {
    INIT,
    IDLE,
    ACTIVE,
    DONE,
    ERROR
};

void main_fsm();

