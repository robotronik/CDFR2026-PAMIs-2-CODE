#pragma once 

enum MainFSM_State {
    INIT,
    IDLE,
    ACTIVE,
    ERROR
};

void main_fsm();

