#include "actuators/leds.h"

Led::Led(gpio_num_t pin) : pin(pin) {
    gpio_config_t default_pull_config = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&default_pull_config);
}

gpio_num_t Led::get_pin() {
    return pin;
}

void StatusLed::toggle() {
    gpio_set_level(get_pin(), !current_state);
    current_state = !current_state;
}
