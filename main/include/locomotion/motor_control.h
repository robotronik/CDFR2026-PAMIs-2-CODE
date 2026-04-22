#pragma once
#include <stdint.h>
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
        bool has_target;

        float lin_integral;
        float lin_prev_error;
        float ang_integral;
        float ang_prev_error;

        int64_t last_control_us;

        static float normalize_angle_deg(float angle_deg);
        static float clamp(float value, float min_value, float max_value);
        void reset_pid();
    public:
        MotorControl();
        void move(coords_t dest);
        void update();
        void start();
        void stop();
};
