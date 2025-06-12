#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "disable_led_strip.h"
#include "motor_pwm.h"

void app_main(void)
{
    disable_led();

    init_pwm_A();
    start_pwm_A();

    init_direction_A();

    while (1) {
        set_pwm_A(100);
        set_direction_A(1);
        vTaskDelay(pdMS_TO_TICKS(2000));  // Delay 1000 ms (1 second)
        set_pwm_A(70);
        set_direction_A(2);
        vTaskDelay(pdMS_TO_TICKS(2000));  // Delay 1000 ms (1 second)
    }
}