#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "utils.h"
#include "imu.h"

static const char *TAG = "imu";

static imu_input_t imu;
static QueueHandle_t uart_tx_queue;

void imu_task(void *param) {
    uart_tx_queue = (QueueHandle_t)param;

    while (1) {
        imu.accel_x = (float)rand() / RAND_MAX;
        imu.accel_y = (float)rand() / RAND_MAX;
        imu.accel_z = (float)rand() / RAND_MAX;

        imu.gyro_x = (float)rand() / RAND_MAX;
        imu.gyro_y = (float)rand() / RAND_MAX;
        imu.gyro_z = (float)rand() / RAND_MAX;

        uart_tx_message_t msg;
        msg.type = MSG_TYPE_IMU;
        msg.size = sizeof(imu);
        memcpy(msg.payload, &imu, sizeof(imu));

        if (xQueueSend(uart_tx_queue, &msg, portMAX_DELAY) != pdTRUE) {
            ESP_LOGI(TAG, "Queue full\n");
        }

        vTaskDelay(500 / portTICK_PERIOD_MS); // 500 ms
    }
}