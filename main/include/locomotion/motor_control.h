#pragma once
#include "locomotion/motor.h"
#include "locomotion/encoder.h"

class MotorControl { 
    private:
        Encoder encoder_a;
        Encoder encoder_b;
        Motor motor_a;
        Motor motor_b;
    public:
        MotorControl();
        void move(float a, float b);
        void start();
        void stop();
};
