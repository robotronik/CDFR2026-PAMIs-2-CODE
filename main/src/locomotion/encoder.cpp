#include "esp_err.h"
#include "pins.h"
#include "locomotion/encoder.h"

// TODO: count method and glitch filter?

Encoder::Encoder(gpio_num_t pin_a, gpio_num_t pin_b) {
    /* Unit setup */
    pcnt_unit_config_t unit_config = {
        .clk_src = PCNT_CLK_SRC_DEFAULT,
        .low_limit = -100, // todo check needed cnt resolution 
        .high_limit = 100,
        .intr_priority = 0,
        .flags = {}
    };
    pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
    
    /* Channel setup */
    pcnt_chan_config_t chan_config_a = {
        .edge_gpio_num = pin_a,
        .level_gpio_num = pin_b,
        .flags = {}, 
    };
    pcnt_chan_config_t chan_config_b = {
        .edge_gpio_num = pin_b,
        .level_gpio_num = pin_a,
        .flags = {},
    };
    channel_a = NULL;
    channel_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config_a, &channel_a)); 
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config_b, &channel_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(channel_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(channel_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
 
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(channel_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(channel_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE)); 
}

void Encoder::start() {
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

void Encoder::stop() {
    ESP_ERROR_CHECK(pcnt_unit_stop(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_disable(pcnt_unit));
}


