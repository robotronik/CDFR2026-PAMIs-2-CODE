#include "actuators/servo.h"


int Servo::next_channel = 0;

Servo::Servo(gpio_num_t pin) : pin(pin), attached(false), channel(-1) {}

esp_err_t Servo::attach() {
	if (attached) {
		return ESP_OK;
	}

	if (next_channel == 0) {
		ledc_timer_config_t timer_config = {};
		timer_config.speed_mode = SERVO_MODE;
		timer_config.timer_num = SERVO_TIMER;
		timer_config.duty_resolution = SERVO_RESOLUTION;
		timer_config.freq_hz = SERVO_FREQ_HZ;
		timer_config.clk_cfg = LEDC_AUTO_CLK;
		esp_err_t err = ledc_timer_config(&timer_config);
		if (err != ESP_OK) {
			return err;
		}
	}

	if (channel < 0) {
		if (next_channel >= LEDC_CHANNEL_MAX) {
			return ESP_ERR_NOT_FOUND;
		}

		channel = next_channel;
		next_channel++;
	}

	ledc_channel_config_t channel_config = {};
	channel_config.gpio_num = pin;
	channel_config.speed_mode = SERVO_MODE;
	channel_config.channel = static_cast<ledc_channel_t>(channel);
	channel_config.timer_sel = SERVO_TIMER;
	channel_config.duty = pulse_us_to_duty(1500);
	channel_config.hpoint = 0;
	esp_err_t err = ledc_channel_config(&channel_config);
	if (err != ESP_OK) {
		return err;
	}

	attached = true;
	return ESP_OK;
}

esp_err_t Servo::detach() {
	if (!attached) {
		return ESP_OK;
	}

	esp_err_t err = ledc_stop(SERVO_MODE, static_cast<ledc_channel_t>(channel), 0);
	if (err != ESP_OK) {
		return err;
	}

	attached = false;
	return ESP_OK;
}

esp_err_t Servo::write_angle(uint8_t angle_deg) {
	if (!attached) {
		return ESP_ERR_INVALID_STATE;
	}

	if (angle_deg > 180) {
		angle_deg = 180;
	}

	uint32_t pulse_us = SERVO_MIN_US + ((SERVO_MAX_US - SERVO_MIN_US) * angle_deg) / 180;
	return write_us(pulse_us);
}

esp_err_t Servo::write_us(uint32_t pulse_width_us) {
	if (!attached) {
		return ESP_ERR_INVALID_STATE;
	}

	if (pulse_width_us < SERVO_MIN_US) {
		pulse_width_us = SERVO_MIN_US;
	}
	if (pulse_width_us > SERVO_MAX_US) {
		pulse_width_us = SERVO_MAX_US;
	}

	uint32_t duty = pulse_us_to_duty(pulse_width_us);
	esp_err_t err = ledc_set_duty(SERVO_MODE, static_cast<ledc_channel_t>(channel), duty);
	if (err != ESP_OK) {
		return err;
	}
	err = ledc_update_duty(SERVO_MODE, static_cast<ledc_channel_t>(channel));
	if (err != ESP_OK) {
		return err;
	}

	return ESP_OK;
}

bool Servo::is_attached() const {
	return attached;
}

uint32_t Servo::pulse_us_to_duty(uint32_t pulse_us) const {
	constexpr uint32_t period_us = 1000000UL / SERVO_FREQ_HZ;
	constexpr uint32_t max_duty = (1UL << SERVO_RESOLUTION) - 1;
	return static_cast<uint32_t>((static_cast<uint64_t>(pulse_us) * max_duty + (period_us / 2U)) / period_us);
}
