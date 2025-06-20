#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "disable_led_strip.h"
#include "motor_pwm.h"
#include "xbox_ble.h"
#include "jetson_uart.h"
#include "freertos/queue.h"
#include "utils.h"

void app_main(void)
{
    const static char *TAG = "app_main";

    QueueHandle_t xbox_input_queue;
    QueueHandle_t uart_tx_queue = NULL; 

    xbox_input_queue = xQueueCreate(10, sizeof(xbox_report_payload_t));
    if (xbox_input_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue");
    }

    uart_tx_queue = xQueueCreate(20, sizeof(char *));
    if (uart_tx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue");
    }

    /*
    For some reason motor pwm needs to be initalized in app_main() or else there will be error
    So put all initalization in app_main
    */
    // pwm
    init_pwm_dual();
    init_direction_dual();
    // ble
    xbox_ble_init();
    // uart
    uart_init();

    xTaskCreatePinnedToCore(xbox_ble_task, "xbox_ble_task", 4096, xbox_input_queue, 3, NULL, PRO_CPU_NUM);
    xTaskCreatePinnedToCore(motor_pwm_task, "motor_pwm_task", 4096, xbox_input_queue, 3, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(jetson_uart_task, "jetson_uart_task", 4096, uart_tx_queue, 3, NULL, tskNO_AFFINITY);
}

