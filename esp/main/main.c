#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "motor_pwm.h"
#include "xbox_ble.h"
#include "imu.h"
#include "ina.h"
#include "jetson_uart.h"
#include "utils.h"
#include "encoder.h"
#include "i2c_master.h"
#include "esp_log.h"
#include "ultrasonic.h"
// #include "phone_ble.h"
// #include "bluedroid_ble.h"
#include "pid.h"

QueueHandle_t motor_queue;
QueueHandle_t uart_tx_queue;
SemaphoreHandle_t pid_xbox_mutex;
SemaphoreHandle_t pid_encoder_mutex;
motor_input_t pid_xbox_input;
uint16_t pid_encoder_input;

void app_main(void)
{
    const static char *TAG = "app_main";

    motor_queue = xQueueCreate(10, sizeof(motor_input_t));
    if (motor_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue");
    }

    uart_tx_queue = xQueueCreate(30, sizeof(uart_tx_message_t));
    if (uart_tx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue");
    }

    #ifdef CONFIG_INPUT_SOURCE_PID
        pid_xbox_mutex = xSemaphoreCreateMutex();
        if (pid_xbox_mutex == NULL) {
            printf("Failed to create mutex!\n");
            return;
        }

        pid_encoder_mutex = xSemaphoreCreateMutex();
        if (pid_encoder_mutex == NULL) {
            printf("Failed to create mutex!\n");
            return;
        }
    #endif

    pid_xbox_input.pwm = 0;
    pid_xbox_input.direction = 0;
    pid_encoder_input = 0;

    /*
    For some reason motor pwm needs to be initalized in app_main() or else there will be error
    So put all initalization in app_main
    */
    // pwm
    init_pwm();
    init_direction();
    // xbox ble
    xbox_ble_init();
    // uart
    uart_init();
    // wheel encoder
    encoder_init();
    // i2c
    i2c_master_init();
    // imu
    imu_init();
    // ina
    ina_init();
    // ultrasonic
    ultrasonic_init();

    xTaskCreatePinnedToCore(jetson_uart_tx_task, "jetson_uart_tx_task", 4096, NULL, 4, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(xbox_ble_task, "xbox_ble_task", 4096, NULL, 3, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(motor_pwm_task, "motor_pwm_task", 4096, NULL, 4, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(encoder_task, "encoder_task", 4096, NULL, 5, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(imu_task, "imu_task", 4096, NULL, 4, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(ina_task, "ina_task", 4096, NULL, 3, NULL, tskNO_AFFINITY);
    #ifdef CONFIG_INPUT_SOURCE_PID
        xTaskCreatePinnedToCore(pid_task, "pid_task", 4096, NULL, 5, NULL, tskNO_AFFINITY);
    #endif

    // xTaskCreatePinnedToCore(phone_ble_task, "phone_ble_task", 4096, NULL, 3, NULL, PRO_CPU_NUM);
    // xTaskCreatePinnedToCore(jetson_uart_rx_task, "jetson_uart_rx_task", 4096, NULL, 3, NULL, tskNO_AFFINITY);
    // xTaskCreatePinnedToCore(ultrasonic_task, "ultrasonic_task", 4096, NULL, 3, NULL, tskNO_AFFINITY);
}

