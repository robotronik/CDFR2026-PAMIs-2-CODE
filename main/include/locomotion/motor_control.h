#pragma once
#include "locomotion/motor.h"
#include "locomotion/encoder.h"


struct coords {
    float x;
    float y;
    float angle;

    void updateAngle(float newAngle) {
        angle = std::fmod(newAngle, 360.0f);
        if (angle < 0) {
            angle += 360.0f;
        }
    }
};

class MotorControl { 
    private:
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
