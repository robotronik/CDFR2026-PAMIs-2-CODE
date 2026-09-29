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
        bool turn_end;
        bool is_reversed;
        int64_t last_control_us;
        int64_t begin_us;
        
        // Store last error for derivative corrector
        float last_rot_error = 0.0f;
        float last_distance_error = 0.0f;
        float last_heading_error = 0.0f;

        // Filter
        float last_rot_derivative = 0.0f;
        float last_lin_derivative = 0.0f;
        float last_steer_derivative = 0.0f;  
        
        // Approach control
        float approach_angle;

        // Speed ramp 
        float previous_lin_cmd = 0.0f;
        float previous_rot_cmd = 0.0f;
        float previous_steer_cmd = 0.0f;

        // Integrator
        float steer_integral_error = 0.0f;

    public:
        MotorControl();
        void set_coords(coords_t coords);
        coords_t get_coords();
        // Set a new destination and return true if already at destination.
        // If `turnEnd` is true, the controller will also orient to the
        // target angle before reporting arrival.
        bool goTo(coords_t dest, bool turnEnd = false, bool reverse = false);
        // Periodic call to advance motion toward the last-set target.
        // If `turnEnd` is true, the controller will finish only after final rotation.
        // Returns true when the destination has been reached.
        bool goTo(bool turnEnd = false);
        void start();
        void stop();
        void reset_pid();

        // bool INVERTED_LEFT_MOTOR;
};
