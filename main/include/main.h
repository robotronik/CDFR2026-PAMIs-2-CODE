#pragma once 
#include "locomotion/motor_control.h"
#include "actuators/pull_switch.h"
#include "actuators/leds.h"
#include "actuators/servo.h"
#include "actuators/ultrasonic.h"

// Pami 0 is ninja, so we can disable all the code related to it for now. 
// This will make it easier to test the rest of the code
#ifndef N_PAMI
#define N_PAMI 1
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

extern MotorControl motor_control;
extern PullSwitch pull_switch;
extern StatusLed status_led;
extern Servo servo_1;
extern Servo servo_2;
extern Ultrasonic ultrasonic;

void main_fsm();

