#pragma once
#include "driver/gpio.h"

namespace robot_pins {
    // --- Hall Effect sensors ---
    constexpr gpio_num_t PIN_HALL_A1 = GPIO_NUM_32;
    constexpr gpio_num_t PIN_HALL_A2 = GPIO_NUM_33;
    constexpr gpio_num_t PIN_HALL_B1 = GPIO_NUM_36;
    constexpr gpio_num_t PIN_HALL_B2 = GPIO_NUM_39;

    // --- DC Drivers ---
    constexpr gpio_num_t PIN_DC_A1 = GPIO_NUM_17;
    constexpr gpio_num_t PIN_DC_A2 = GPIO_NUM_18;
    constexpr gpio_num_t PIN_DC_B1 = GPIO_NUM_19;
    constexpr gpio_num_t PIN_DC_B2 = GPIO_NUM_21;

    // --- Servomotors ---
    constexpr gpio_num_t PIN_SERVO_1 = GPIO_NUM_26;
    constexpr gpio_num_t PIN_SERVO_2 = GPIO_NUM_27;

    // --- Ultrasonic sensor ---
    constexpr gpio_num_t PIN_US_TX = GPIO_NUM_35;
    constexpr gpio_num_t PIN_US_RX = GPIO_NUM_16;

    // --- Utility ---
    constexpr gpio_num_t PIN_BATTERY_LEVEL = GPIO_NUM_34;
    constexpr gpio_num_t PIN_BOOT          = GPIO_NUM_0;         
    constexpr gpio_num_t PIN_SW_TIRETTE    = GPIO_NUM_23;
    constexpr gpio_num_t PIN_SW_TEAM       = GPIO_NUM_25;
    constexpr gpio_num_t PIN_TEAM_RGB      = GPIO_NUM_22;
    constexpr gpio_num_t PIN_STATUS_LED    = GPIO_NUM_4;
}

