#pragma once 
#include "locomotion/motor_control.h"
#include "actuators/switch.h"
#include "actuators/leds.h"
#include "actuators/servo.h"
#include "actuators/ultrasonic.h"
#include "battery_monitor.h"

// 0 error only
// 1 info 
// 2 debug activated 
#define DEBUG_LEVEL 2

// Pami 0 is ninja, so we can disable all the code related to it for now. 
// This will make it easier to test the rest of the code
#ifndef N_PAMI
#define N_PAMI 0
#endif

#if N_PAMI == 0
#define NINJA
#endif

enum MainFSMState {
    INIT,
    IDLE,
    ACTIVE,
    DONE,
    ERROR
};

enum Team {
    YELLOW,
    BLUE
};

extern MotorControl motor_control;
extern Switch pull_switch;
extern Switch team_switch;
extern StatusLed status_led;
extern TeamLed team_led;
extern Servo servo_1;
extern Servo servo_2;
extern Ultrasonic ultrasonic;
extern BatteryMonitor battery_monitor;

extern Team current_team;

void main_fsm();

