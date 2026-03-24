#include "locomotion/motor.h"

Motor::Motor(int pin_in1, int pin_in2) {
    /* Timer setup */
    mcpwm_timer_config_t timer_config = {};
    timer_config.group_id = 0; 
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = 20000000; 
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    timer_config.period_ticks = 1000; 
    
    mcpwm_new_timer(&timer_config, &timer);
 
    /* Operator setup */
    mcpwm_operator_config_t oper_config = {};
    oper_config.group_id = 0;
    mcpwm_new_operator(&oper_config, &oper);
    mcpwm_operator_connect_timer(oper, timer);

    /* Comparator setup */
    mcpwm_comparator_config_t cmpr_config = {};
    cmpr_config.flags.update_cmp_on_tez = true;
    
    mcpwm_new_comparator(oper, &cmpr_config, &cmpr1);
    mcpwm_new_comparator(oper, &cmpr_config, &cmpr2);

    /* Generator setup */
    mcpwm_generator_config_t gen1_config = {};
    gen1_config.gen_gpio_num = pin_in1;
    mcpwm_new_generator(oper, &gen1_config, &gen1);
    
    mcpwm_generator_config_t gen2_config = {};
    gen2_config.gen_gpio_num = pin_in2;
    mcpwm_new_generator(oper, &gen2_config, &gen2);

    /* Enable timer */
    mcpwm_generator_set_action_on_timer_event(gen1, 
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_timer_event(gen2, 
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
        
    mcpwm_generator_set_action_on_compare_event(gen1, 
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr1, MCPWM_GEN_ACTION_LOW));
    mcpwm_generator_set_action_on_compare_event(gen2, 
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr2, MCPWM_GEN_ACTION_LOW));
 
    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
}


    

