#include "actuators/ultrasonic.h"

#include "esp_rom_sys.h"
#include "esp_timer.h"

Ultrasonic::Ultrasonic(gpio_num_t trig_pin, gpio_num_t echo_pin)
	: trig_pin(trig_pin),
	  echo_pin(echo_pin),
	  initialized(false),
	  has_last_distance(false),
	  last_distance_mm(0),
	  last_trigger_us(0) {}

esp_err_t Ultrasonic::init() {
	if (!GPIO_IS_VALID_GPIO(trig_pin) || !GPIO_IS_VALID_OUTPUT_GPIO(trig_pin)) {
		return ESP_ERR_INVALID_ARG;
	}
	if (!GPIO_IS_VALID_GPIO(echo_pin)) {
		return ESP_ERR_INVALID_ARG;
	}

	gpio_config_t trig_cfg = {};
	trig_cfg.pin_bit_mask = (1ULL << trig_pin);
	trig_cfg.mode = GPIO_MODE_OUTPUT;
	trig_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
	trig_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
	trig_cfg.intr_type = GPIO_INTR_DISABLE;
	esp_err_t err = gpio_config(&trig_cfg);
	if (err != ESP_OK) {
		return err;
	}

	gpio_config_t echo_cfg = {};
	echo_cfg.pin_bit_mask = (1ULL << echo_pin);
	echo_cfg.mode = GPIO_MODE_INPUT;
	echo_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
	echo_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
	echo_cfg.intr_type = GPIO_INTR_DISABLE;
	err = gpio_config(&echo_cfg);
	if (err != ESP_OK) {
		return err;
	}

	err = gpio_set_level(trig_pin, 0);
	if (err != ESP_OK) {
		return err;
	}

	initialized = true;
	return ESP_OK;
}

esp_err_t Ultrasonic::read_distance_mm(uint32_t& distance_mm, uint32_t timeout_us) {
	if (!initialized) {
		return ESP_ERR_INVALID_STATE;
	}

	const int64_t now_us = esp_timer_get_time();
	if ((now_us - last_trigger_us) < static_cast<int64_t>(MIN_TRIGGER_PERIOD_US)) {
		if (has_last_distance) {
			distance_mm = last_distance_mm;
			return ESP_OK;
		}
		return ESP_ERR_TIMEOUT;
	}

	int64_t ts_unused = 0;
	int64_t pulse_start_us = 0;
	int64_t pulse_end_us = 0;

	esp_err_t err = wait_for_level(0, PRE_TRIGGER_LOW_TIMEOUT_US, ts_unused);
	if (err != ESP_OK) {
		if (has_last_distance) {
			distance_mm = last_distance_mm;
			return ESP_OK;
		}
		return err;
	}

	gpio_set_level(trig_pin, 0);
	esp_rom_delay_us(2);
	gpio_set_level(trig_pin, 1);
	esp_rom_delay_us(10);
	gpio_set_level(trig_pin, 0);
	last_trigger_us = esp_timer_get_time();

	err = wait_for_level(1, timeout_us, pulse_start_us);
	if (err != ESP_OK) {
		if (has_last_distance) {
			distance_mm = last_distance_mm;
			return ESP_OK;
		}
		return err;
	}

	err = wait_for_level(0, timeout_us, pulse_end_us);
	if (err != ESP_OK) {
		if (has_last_distance) {
			distance_mm = last_distance_mm;
			return ESP_OK;
		}
		return err;
	}

	if (pulse_end_us <= pulse_start_us) {
		if (has_last_distance) {
			distance_mm = last_distance_mm;
			return ESP_OK;
		}
		return ESP_ERR_INVALID_RESPONSE;
	}

	const uint32_t pulse_width_us = static_cast<uint32_t>(pulse_end_us - pulse_start_us);
	distance_mm = (pulse_width_us * 343U) / 2000U;
	last_distance_mm = distance_mm;
	has_last_distance = true;

	return ESP_OK;
}

esp_err_t Ultrasonic::read_distance_cm(float& distance_cm, uint32_t timeout_us) {
	uint32_t distance_mm = 0;
	esp_err_t err = read_distance_mm(distance_mm, timeout_us);
	if (err != ESP_OK) {
		return err;
	}

	distance_cm = static_cast<float>(distance_mm) / 10.0f;
	return ESP_OK;
}

esp_err_t Ultrasonic::wait_for_level(int level, uint32_t timeout_us, int64_t& timestamp_us) const {
	const int64_t start_us = esp_timer_get_time();
	const int64_t deadline_us = start_us + static_cast<int64_t>(timeout_us);

	while (gpio_get_level(echo_pin) != level) {
		if (esp_timer_get_time() >= deadline_us) {
			return ESP_ERR_TIMEOUT;
		}
	}

	timestamp_us = esp_timer_get_time();
	return ESP_OK;
}
