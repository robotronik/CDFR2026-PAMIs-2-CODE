#pragma once
#include "locomotion/motor.h"
#include "locomotion/encoder.h"
#include "rtos_wrapper.h"
#include "structs.h"
#include <mutex>

enum MotorControlState {
    ROTATION,
    LINEAR,
    START,
    STOP
};

class MotorControl { 
    private:
        SharedPos& shared_pos;

        Encoder encoder_a;
        Encoder encoder_b;
        Motor motor_a;
        Motor motor_b;

        coords target;

        
    public:
        MotorControl();
        void move(coords dest);
        void start();
        void stop();
};
