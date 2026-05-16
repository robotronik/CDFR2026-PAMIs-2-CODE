#include "locomotion/motor.h"
#include "hal/mcpwm_types.h"
#include "structs.h"

#define IS_MONO TRUE

namespace {
    // PID constants for motor percentage.
    constexpr float KP = 1.5f;

    #ifndef IS_MONO
    constexpr float KI = 0.0f;
    #else   
    constexpr float KI = 25.0f;
    #endif

    constexpr float MAX_TICKS_PER_SECOND = 2700.0f; // TODO : measure the real max tick speed
}

Motor::Motor(gpio_num_t motor_pin_a, gpio_num_t motor_pin_b, gpio_num_t encoder_pin_a, gpio_num_t encoder_pin_b)
    : encoder(encoder_pin_a, encoder_pin_b) {
    /* Timer setup */
    mcpwm_timer_config_t timer_config = {};
    timer_config.group_id = 0; 
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = 20000000; 
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    timer_config.period_ticks = 1000; 
    
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));
 
    /* Operator setup */
    mcpwm_operator_config_t oper_config = {};
    oper_config.group_id = 0;
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    /* Comparator setup */
    mcpwm_comparator_config_t cmpr_config = {};
    cmpr_config.flags.update_cmp_on_tez = true;
    
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmpr_config, &cmpr1));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmpr_config, &cmpr2));

    /* Generator setup */
    mcpwm_generator_config_t gen1_config = {};
    gen1_config.gen_gpio_num = motor_pin_a;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen1_config, &gen1));
    
    mcpwm_generator_config_t gen2_config = {};
    gen2_config.gen_gpio_num = motor_pin_b;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen2_config, &gen2));

    /* Enable timer */
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen1, 
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen2, 
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen1, 
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr1, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen2, 
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr2, MCPWM_GEN_ACTION_LOW)));
}

// Returns the number of turns
float Motor::get_delta() {
    // Calculate the speed of the motor in ticks per second
    float delta_ticks = this->encoder.get_delta();
    float turns = 2.0f * delta_ticks / 1050.0f; // 1050 is the number of ticks per wheel turn (depends on the encoder and gearing)
    float speed_percent = (delta_ticks / MAX_TICKS_PER_SECOND) * 10000.0f;
    this->filtered_speed = this->alpha * speed_percent + (1.0f - this->alpha) * this->filtered_speed;
    return turns;
}

void Motor::set_speed_pid(float percentage) {
    
    const int64_t now_us = esp_timer_get_time();
    float dt_s = 0.01f;
    if (this->last_control_us != 0) {
        dt_s = (now_us - this->last_control_us) / 1000000.0f;
        dt_s = clamp(dt_s, 0.001f, 0.1f);
    }
    this->last_control_us = now_us;

    float error = percentage - this->filtered_speed;

    this->integral_error += error * dt_s;

    float max_integral = 100.0f; // Evite l'embalement
    if (this->integral_error > max_integral) {
        this->integral_error = max_integral;
    } else if (this->integral_error < -max_integral) {
        this->integral_error = -max_integral;
    }

    float command_speed_percent = (KP * error) + (KI * this->integral_error);
  
    this->set_speed(command_speed_percent);    
}

void Motor::set_speed(float percentage) {
    if (percentage > 100.0f) {
        percentage = 100.0f;
    } else if (percentage < -100.0f) {
        percentage = -100.0f;
    }

    uint32_t compare_val = (uint32_t)((abs(percentage) * 10.0));

    if (percentage > 0.0f) {  
        mcpwm_comparator_set_compare_value(this->cmpr1, compare_val);
        mcpwm_comparator_set_compare_value(this->cmpr2, 0);
    } else if (percentage < 0.0f) {
        mcpwm_comparator_set_compare_value(this->cmpr1, 0);
        mcpwm_comparator_set_compare_value(this->cmpr2, compare_val);
    } else { 
        mcpwm_comparator_set_compare_value(this->cmpr1, 0);
        mcpwm_comparator_set_compare_value(this->cmpr2, 0);
    }
}

void Motor::start() {
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
    encoder.start();
}

void Motor::stop() {
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY));
    ESP_ERROR_CHECK(mcpwm_timer_disable(timer));
    encoder.stop();
}

void Motor::reset_pid() {
    integral_error = 0.0f;
    prev_error = 0.0f;
    last_control_us = 0;
    filtered_speed = 0.0f;
}

    

