#include "locomotion/motor.h"
#include "hal/mcpwm_types.h" 
using namespace robot_pins;

Motor::Motor(gpio_num_t pin_a, gpio_num_t pin_b) : current_speed(0) {
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
    gen1_config.gen_gpio_num = pin_a;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen1_config, &gen1));
    
    mcpwm_generator_config_t gen2_config = {};
    gen2_config.gen_gpio_num = pin_b;
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

void Motor::set_speed(float percentage) {  
        uint32_t compare_val = (uint32_t)((abs(percentage) * 0.001));
        current_speed = percentage;

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
}

void Motor::stop() {
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY));
    ESP_ERROR_CHECK(mcpwm_timer_disable(timer));
}

    

