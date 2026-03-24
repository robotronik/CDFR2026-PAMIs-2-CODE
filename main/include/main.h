#pragma once 
#include "locomotion/motor_control.h"

enum MainFSM_State {
    INIT,
    IDLE,
    ACTIVE,
    ERROR
};

void main_fsm();

