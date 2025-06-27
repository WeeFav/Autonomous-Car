#include <stdio.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "motor_pwm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "utils.h"
#include "freertos/queue.h"

#define MOTOR_STOP 0
#define MOTOR_FORWARD 1
#define MOTOR_BACKWARD 2
#define ENA_GPIO 40
#define ENB_GPIO 41
#define IN1_GPIO 4
#define IN2_GPIO 5
#define IN3_GPIO 6
#define IN4_GPIO 7

static const char *TAG = "motor_pwm";

static mcpwm_timer_handle_t timer;
static mcpwm_oper_handle_t operators;
static mcpwm_cmpr_handle_t comparatorA;
static mcpwm_cmpr_handle_t comparatorB;
static mcpwm_gen_handle_t generatorA;
static mcpwm_gen_handle_t generatorB;

void init_pwm()
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
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operators));
    ESP_LOGI(TAG, "Connect operators to the same timer");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators, timer));

    // Comparator A //
    ESP_LOGI(TAG, "Create comparators");
    mcpwm_comparator_config_t compare_config = {
        .flags.update_cmp_on_tez = true,
    };    
    ESP_ERROR_CHECK(mcpwm_new_comparator(operators, &compare_config, &comparatorA));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparatorA, 0)); 

    // Comparator B //
    ESP_ERROR_CHECK(mcpwm_new_comparator(operators, &compare_config, &comparatorB));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparatorB, 0)); 

    // Generator A //
    ESP_LOGI(TAG, "Create generators");
    mcpwm_generator_config_t gen_configA = {
        .gen_gpio_num = ENA_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(operators, &gen_configA, &generatorA));

    // Generator B //
    mcpwm_generator_config_t gen_configB = {
        .gen_gpio_num = ENB_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(operators, &gen_configB, &generatorB));

    // Generator Action //
    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // PWM Start with HIGH State when Timer is 0 and LOW State when Comparators value is equal Timer, For MCPWM_TIMER_COUNT_MODE_UP
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generatorA, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generatorA, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparatorA, MCPWM_GEN_ACTION_LOW)));   
    
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generatorB, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generatorB, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparatorB, MCPWM_GEN_ACTION_LOW)));   

    // Start PWM
    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));   
}

void init_direction() {
    gpio_reset_pin(IN1_GPIO);
    gpio_reset_pin(IN2_GPIO);
    gpio_reset_pin(IN3_GPIO);
    gpio_reset_pin(IN4_GPIO);
    gpio_set_direction(IN1_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN2_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN3_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN4_GPIO, GPIO_MODE_OUTPUT);
}

static void set_pwm_A(int duty_cycle) {
    if (duty_cycle < 0) {
        duty_cycle = 0;
    }
    else if (duty_cycle > 100) {
        duty_cycle = 100;
    }
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparatorA, duty_cycle * 10)); 
}

static void set_pwm_B(int duty_cycle) {
    if (duty_cycle < 0) {
        duty_cycle = 0;
    }
    else if (duty_cycle > 100) {
        duty_cycle = 100;
    }
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparatorB, duty_cycle * 10)); 
}

static void set_direction_A(int dir) {
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

static void set_direction_B(int dir) {
    if (dir == 1) {
        gpio_set_level(IN3_GPIO, 1);
        gpio_set_level(IN4_GPIO, 0);
    }
    else if (dir == 2) {
        gpio_set_level(IN3_GPIO, 0);
        gpio_set_level(IN4_GPIO, 1);        
    }
    else {
        gpio_set_level(IN3_GPIO, 0);
        gpio_set_level(IN4_GPIO, 0);
    }
}

void motor_pwm_task(void *param) {
    QueueHandle_t xbox_input_queue = (QueueHandle_t)param;
    xbox_input_t report;

    while (1) {
        if (xQueueReceive(xbox_input_queue, (void *)&report, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "D-Pad: %u\n", report.dpad);
            if (report.dpad == 1) {
                set_pwm_A(100);
                set_direction_A(MOTOR_FORWARD);
                set_pwm_B(100);
                set_direction_B(MOTOR_FORWARD);
            }
            else if (report.dpad == 3) {
                set_pwm_A(100);
                set_direction_A(MOTOR_BACKWARD);
                set_pwm_B(100);
                set_direction_B(MOTOR_FORWARD);
            }
            else if (report.dpad == 5) {
                set_pwm_A(100);
                set_direction_A(MOTOR_BACKWARD);
                set_pwm_B(100);
                set_direction_B(MOTOR_BACKWARD);
            }
            else if (report.dpad == 7) {
                set_pwm_A(100);
                set_direction_A(MOTOR_FORWARD);
                set_pwm_B(100);
                set_direction_B(MOTOR_BACKWARD);
            }
            else {
                set_pwm_A(0);
                set_direction_A(MOTOR_STOP);
                set_pwm_B(0);
                set_direction_B(MOTOR_STOP);
            }
        } 
        else {
            ESP_LOGI(TAG, "Failed to received xbox controller input");
        }
    }
}
