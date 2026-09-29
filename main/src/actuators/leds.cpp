#include "actuators/leds.h"
#include <esp_log.h>

static const char* LOGGER_TAG = "Led";

Led::Led(gpio_num_t pin) : pin(pin) {
    gpio_config_t default_pull_config = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&default_pull_config));
}

void StatusLed::toggle() {
    ESP_ERROR_CHECK(gpio_set_level(pin, !current_state));
    current_state = !current_state;
}

void StatusLed::set(bool on) {
    ESP_ERROR_CHECK(gpio_set_level(pin, on));
    current_state = on;
}

TeamLed::TeamLed(gpio_num_t pin) : Led(pin) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = pin,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false,
        }
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags = {
            .with_dma = false
        }
    };

    strip_handle = NULL;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &strip_handle));
}

void TeamLed::set_color(int r, int g, int b) {
    ESP_LOGI(LOGGER_TAG, "Set color to %d %d %d", r, g, b);
    led_strip_set_pixel(strip_handle, 0, r, g, b);
    led_strip_refresh(strip_handle);
}
