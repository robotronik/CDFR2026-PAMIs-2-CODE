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
        int64_t last_control_us;
        
    public:
        MotorControl();
            // Set a new destination and return true if already at destination.
            // If `turnEnd` is true, the controller will also orient to the
            // target angle before reporting arrival.
            bool goTo(coords_t dest, bool turnEnd = false);
            // Periodic call to advance motion toward the last-set target.
            // If `turnEnd` is true, the controller will finish only after final rotation.
            // Returns true when the destination has been reached.
            bool goTo(bool turnEnd = false);
        void start();
        void stop();
        bool target_reached();
};
