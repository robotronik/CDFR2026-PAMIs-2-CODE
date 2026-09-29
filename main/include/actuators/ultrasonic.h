#pragma once

#include "driver/gpio.h"
#include "esp_err.h"
#include "pins.h"

class Ultrasonic {
	private:
		gpio_num_t trig_pin;
		gpio_num_t echo_pin;
		bool initialized;
		bool has_last_distance;
		uint32_t last_distance_mm;
		int64_t last_trigger_us;

		static constexpr uint32_t MIN_TRIGGER_PERIOD_US = 60000;
		static constexpr uint32_t PRE_TRIGGER_LOW_TIMEOUT_US = 2000;

		esp_err_t wait_for_level(int level, uint32_t timeout_us, int64_t& timestamp_us) const;

	public:
		Ultrasonic(gpio_num_t trig_pin, gpio_num_t echo_pin);

		esp_err_t init();
		esp_err_t read_distance_mm(uint32_t& distance_mm, uint32_t timeout_us = 15000);
		esp_err_t read_distance_cm(float& distance_cm, uint32_t timeout_us = 15000);
};
