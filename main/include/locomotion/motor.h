#pragma once
#include "driver/mcpwm_prelude.h"
#include "pins.h"

class Motor {
    private: 
        mcpwm_timer_handle_t timer;
        mcpwm_oper_handle_t oper;
        mcpwm_cmpr_handle_t cmpr1;
        mcpwm_cmpr_handle_t cmpr2;
        mcpwm_gen_handle_t gen1;
        mcpwm_gen_handle_t gen2;
       
    public: 
        Motor(gpio_num_t pin_a, gpio_num_t pin_b);
        void set_speed(float percentage);
        void start();
        void stop();

        float current_speed;
};




