#include "actuators/pull_switch.h"
#include "pins.h"

using namespace robot_pins;

PullSwitch::PullSwitch(gpio_num_t pin) : pin(pin) {
    gpio_config_t default_pull_config = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE, 
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&default_pull_config));
}

int PullSwitch::read() {
   return gpio_get_level(PIN_SW_TIRETTE) ? 1 : 0;
}
