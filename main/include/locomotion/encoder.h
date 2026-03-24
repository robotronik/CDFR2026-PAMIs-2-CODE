#pragma once
#include "driver/pcnt.h"
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
};
