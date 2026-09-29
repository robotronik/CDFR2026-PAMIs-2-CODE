#pragma once
#include "pins.h"

class Switch {
    private:
        gpio_num_t pin;
    public:
        Switch(gpio_num_t pin);
        int read();
};
