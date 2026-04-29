#pragma once
#include <stdint.h>
#include "locomotion/motor.h"
#include "structs.h"

enum MotorControlState {
    ROTATION,
    LINEAR,
    START,
    STOP
};

class MotorControl { 
    private:
        Motor motor_a;
        Motor motor_b;

        coords_t target_pos;
        coords_t current_pos;
        bool has_target;
        bool doing_final_rotation;

        float lin_integral;
        float lin_prev_error;
        float ang_integral;
        float ang_prev_error;

        int64_t last_control_us;

        void reset_pid();
    public:
        MotorControl();
        void move(coords_t dest);
        void update();
        void start();
        void stop();
        bool target_reached();
};
