#pragma once
#include "pins.h"

class PullSwitch {
    private:
        gpio_num_t pin;
    public:
        PullSwitch(gpio_num_t pin);
        int read();
};
