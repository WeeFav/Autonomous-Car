#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "disable_led_strip.h"
#include "motor_pwm.h"
#include "xbox_ble.h"

void app_main(void)
{
    // For some reason motor pwm needs to be initalized in app_main() or else there will be error
    init_pwm_dual();
    start_pwm_dual();
    init_direction_dual();

    xTaskCreatePinnedToCore(xbox_ble_task, "xbox_ble_task", 4096, NULL, 3, NULL, PRO_CPU_NUM);
    xTaskCreatePinnedToCore(motor_pwm_task, "motor_pwm_task", 4096, NULL, 3, NULL, tskNO_AFFINITY);
}

