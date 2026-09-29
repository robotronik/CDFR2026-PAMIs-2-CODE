#pragma once
#include "pins.h"
#include "led_strip.h"

class Led {
    protected:
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
        void set(bool on);
} ;

class TeamLed: public Led {
    private:
        led_strip_handle_t strip_handle;
    public:
        TeamLed(gpio_num_t pin);
        void set_color(int r, int g, int b);
};
