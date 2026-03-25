#pragma once
#include "pins.h"
#include "led_strip.h"

class Led {
    private:
        gpio_num_t pin;
    public:
        Led(gpio_num_t pin); 
};

class StatusLed: public Led {
    private:
        int current_state = 0;
    public:
        using Led::Led;
        void toggle();
} ;

class TeamLed: public Led {
    // TODO
};
