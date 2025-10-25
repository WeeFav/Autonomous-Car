#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "utils.h"
#include "imu.h"
#include "driver/i2c_master.h"

#define MPU6050_ADDR 0x68

static const char *TAG = "imu";
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

void imu_init() {
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(I2C_NUM_0, &bus_handle));

    // Configure imu device
    i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = MPU6050_ADDR,
    .scl_speed_hz = 100000, // 100kHz
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
}

void imu_task(void *param) {
    QueueHandle_t uart_tx_queue = (QueueHandle_t)param;
    
    // Write to imu to configure it
    // Configure power mode
    uint8_t write_buf[2] = {0x6B, 0x00}; // The packet must have an address followed by the data
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), -1));

    // Configure Gyroscope
    write_buf[0] = 0x1B;
    write_buf[1] = 0x08;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), -1));

    // Configure Accelerometer
    write_buf[0] = 0x1C;
    write_buf[1] = 0x00;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), -1));
    
    // Read 
    write_buf[0] = 0x3B;
    uint8_t raw_imu[14];
    uart_tx_message_t msg;
    imu_input_t imu;

    while (1) {
        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, write_buf, 1, raw_imu, sizeof(raw_imu), -1));
        
        int16_t accel_x = (int16_t)((raw_imu[0] << 8) | raw_imu[1]);
        int16_t accel_y = (int16_t)((raw_imu[2] << 8) | raw_imu[3]);
        int16_t accel_z = (int16_t)((raw_imu[4] << 8) | raw_imu[5]);
        int16_t temp_raw = (int16_t)((raw_imu[6] << 8) | raw_imu[7]);
        int16_t gyro_x = (int16_t)((raw_imu[8] << 8) | raw_imu[9]);
        int16_t gyro_y = (int16_t)((raw_imu[10] << 8) | raw_imu[11]);
        int16_t gyro_z = (int16_t)((raw_imu[12] << 8) | raw_imu[13]);

        imu.accel_x = accel_x / 16384.0;
        imu.accel_y = accel_y / 16384.0;
        imu.accel_z = accel_z / 16384.0;
        imu.gyro_x = gyro_x / 65.5;
        imu.gyro_y = gyro_y / 65.5;
        imu.gyro_z = gyro_z / 65.5;

        // send imu data to UART
        // msg.type = MSG_TYPE_IMU;
        // msg.size = sizeof(imu);
        // memcpy(msg.payload, &imu, sizeof(imu));

        // if (xQueueSend(uart_tx_queue, &msg, portMAX_DELAY) != pdTRUE) {
        //     ESP_LOGI(TAG, "Queue full\n");
        // }

        ESP_LOGI("IMU", "Accel: X=%.2f, Y=%.2f, Z=%.2f | Gyro: X=%.2f, Y=%.2f, Z=%.2f",
                imu.accel_x, imu.accel_y, imu.accel_z,
                imu.gyro_x, imu.gyro_y, imu.gyro_z);

        vTaskDelay(500 / portTICK_PERIOD_MS); // 500 ms
    }
}