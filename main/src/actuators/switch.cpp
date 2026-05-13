#include "actuators/switch.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

Switch::Switch(gpio_num_t pin) : pin(pin) {
    gpio_config_t default_pull_config = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&default_pull_config));
}

int Switch::read() {
    return gpio_get_level(pin) ? 0 : 1;
}
