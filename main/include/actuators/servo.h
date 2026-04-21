#pragma once

#include "pins.h"
#include "driver/ledc.h"
#include "esp_err.h"

class Servo {
	private:
		gpio_num_t pin;
		bool attached;
		int channel;

		static int next_channel;

		static constexpr uint32_t SERVO_FREQ_HZ = 50;
		static constexpr ledc_timer_t SERVO_TIMER = LEDC_TIMER_0;
		static constexpr ledc_mode_t SERVO_MODE = LEDC_LOW_SPEED_MODE;
		// 14-bit is a safer cross-target LEDC configuration at 50Hz.
		static constexpr ledc_timer_bit_t SERVO_RESOLUTION = LEDC_TIMER_14_BIT;
		// Most hobby servos expect a 1ms..2ms pulse range.
		static constexpr uint32_t SERVO_MIN_US = 500;
		static constexpr uint32_t SERVO_MAX_US = 2500;

		uint32_t pulse_us_to_duty(uint32_t pulse_us) const;

	public:
		explicit Servo(gpio_num_t pin);
		esp_err_t attach();
		esp_err_t detach();
		esp_err_t write_angle(uint8_t angle_deg);
		esp_err_t write_us(uint32_t pulse_width_us);
		bool is_attached() const;
};
