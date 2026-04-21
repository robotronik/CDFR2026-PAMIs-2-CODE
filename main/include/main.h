#pragma once 
#include "locomotion/motor_control.h"
#include "actuators/pull_switch.h"
#include "actuators/leds.h"

enum MainFSM_State {
    INIT,
    IDLE,
    ACTIVE,
    DONE,
    ERROR
};

void main_fsm();

