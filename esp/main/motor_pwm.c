#include <stdio.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "motor_pwm.h"

static const char *TAG = "motor_pwm";

int ENA_GPIO = 40;
int IN1_GPIO = 4;
int IN2_GPIO = 5;

mcpwm_timer_handle_t timer = NULL;
mcpwm_cmpr_handle_t comparators = NULL;

void init_pwm_A(void)
{
    // Timer //
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT, // PLL Clock 160 MHz
        .resolution_hz = 10000000, //10 MHz Prescaler, 0.1us/ticks
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = 1000, // 1000 ticks for 1 cycle, 10 MHz/1000 = 10 KHz Timer
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    // Operator //
    ESP_LOGI(TAG, "Create operators");
    mcpwm_oper_handle_t operators = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operators));
    ESP_LOGI(TAG, "Connect operators to the same timer");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators, timer));

    // Comparators //
    ESP_LOGI(TAG, "Create comparators");
    mcpwm_comparator_config_t compare_config = {
        .flags.update_cmp_on_tez = true,
    };    
    ESP_ERROR_CHECK(mcpwm_new_comparator(operators, &compare_config, &comparators));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators, 0)); 

    // Generators //
    ESP_LOGI(TAG, "Create generators");
    mcpwm_gen_handle_t generators = NULL;
    mcpwm_generator_config_t gen_config = {
        .gen_gpio_num = ENA_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(operators, &gen_config, &generators));

    // Generator Action //
    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // PWM Start with HIGH State when Timer is 0 and LOW State when Comparators value is equal Timer, For MCPWM_TIMER_COUNT_MODE_UP
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generators, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generators, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators, MCPWM_GEN_ACTION_LOW)));   

}

void set_pwm_A(int duty_cycle) {
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators, duty_cycle * 10)); 
}

void start_pwm_A(void) {
    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));   
}

void init_direction_A(void) {
    gpio_reset_pin(IN1_GPIO);
    gpio_reset_pin(IN2_GPIO);
    gpio_set_direction(IN1_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN2_GPIO, GPIO_MODE_OUTPUT);
}

void set_direction_A(int dir) {
    if (dir == 1) {
        gpio_set_level(IN1_GPIO, 1);
        gpio_set_level(IN2_GPIO, 0);
    }
    else if (dir == 2) {
        gpio_set_level(IN1_GPIO, 0);
        gpio_set_level(IN2_GPIO, 1);        
    }
    else {
        gpio_set_level(IN1_GPIO, 0);
        gpio_set_level(IN2_GPIO, 0);
    }
}
