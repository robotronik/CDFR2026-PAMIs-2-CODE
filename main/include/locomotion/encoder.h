#pragma once
#include "driver/pulse_cnt.h"
#include "pins.h"

class Encoder {
    private:
        pcnt_unit_handle_t pcnt_unit;
        pcnt_channel_handle_t channel_a;
        pcnt_channel_handle_t channel_b;
    public:
        Encoder(gpio_num_t pin_a, gpio_num_t pin_b);
        void start();
        void stop();
        float get_delta(){/* TODO*/ return 0.0f;};
};
