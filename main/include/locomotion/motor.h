#pragma once
#include "driver/mcpwm_prelude.h"
#include "pins.h"
#include "locomotion/motor.h"
#include "locomotion/encoder.h"
#include <esp_timer.h>

class Motor {
    private: 
        mcpwm_timer_handle_t timer;
        mcpwm_oper_handle_t oper;
        mcpwm_cmpr_handle_t cmpr1;
        mcpwm_cmpr_handle_t cmpr2;
        mcpwm_gen_handle_t gen1;
        mcpwm_gen_handle_t gen2;

        const float alpha = 0.4f; // c'est arbitraire
        float prev_ticks = 0.0f;
        int64_t last_control_us = 0;
        float integral_error = 0.0f;
        float prev_error = 0.0f;
        Encoder encoder;

    public: 
        Motor(gpio_num_t motor_pin_a, gpio_num_t motor_pin_b, gpio_num_t encoder_pin_a, gpio_num_t encoder_pin_b);
        void start();
        void stop();
        void set_speed_pid(float percentage);
        void set_speed(float percentage);
        float get_delta();
        void reset_pid();

        float filtered_speed = 0.0f; // out of 100%
};




