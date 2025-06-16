#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "disable_led_strip.h"
#include "motor_pwm.h"

void app_main(void)
{
    // disable_led();

    init_pwm_dual();
    start_pwm_dual();
    init_direction_dual();

    while (1) {
        set_pwm_A(100);
        set_pwm_B(100);
        set_direction_A(MOTOR_FORWARD);
        set_direction_B(MOTOR_FORWARD);
        vTaskDelay(pdMS_TO_TICKS(5000));  // Delay 1000 ms (1 second)

        set_pwm_A(100);
        set_pwm_B(100);
        set_direction_A(MOTOR_BACKWARD);
        set_direction_B(MOTOR_BACKWARD);
        vTaskDelay(pdMS_TO_TICKS(5000));  // Delay 1000 ms (1 second)
    }
}