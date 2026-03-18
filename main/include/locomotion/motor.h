#pragma once
#include "driver/mcpwm.h"

class Motor {
    private: 
        mcpwm_timer_handle_t timer;
        mcpwm_oper_handle_t oper;
        mcpwm_cmpr_handle_t cmpr1;
        mcpwm_cmpr_handle_t cmpr2
        mcpwm_gen_handle_t gen1;
        mcpwm_gen_handle_t gen2;

    public: 
        Motor(int pin_in1, int pin_in2);
};




