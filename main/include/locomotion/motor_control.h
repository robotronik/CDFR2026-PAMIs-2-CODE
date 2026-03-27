#pragma once
#include "locomotion/motor.h"
#include "locomotion/encoder.h"

struct coords_t {
    float x;
    float y;
    float angle;

    void updateAngle(float newAngle) {
        angle = newAngle;
        if (angle < 0) {
            angle += 360.0f;
        }
    }
};

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

        MotorControlState current_state;

    public:
        MotorControl();
        void move(coords_t new_target);
        void update();
        void start();
        void stop();
};
