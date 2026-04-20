#pragma once
#include "locomotion/motor.h"
#include "locomotion/encoder.h"
#include "structs.h"

enum MotorControlState {
    ROTATION,
    LINEAR,
    START,
    STOP
};

class MotorControl { 
    private:
        Encoder encoder_a;
        Encoder encoder_b;
        Motor motor_a;
        Motor motor_b;

        coords_t target_pos;
        coords_t current_pos;
    public:
        MotorControl();
        void move(coords_t dest);
        void update();
        void start();
        void stop();
};
