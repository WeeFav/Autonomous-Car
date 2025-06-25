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

#define SDA_GPIO 8
#define SCL_GPIO 9
#define MPU6050_ADDR 0x68

static const char *TAG = "imu";

static imu_input_t imu;
static QueueHandle_t uart_tx_queue;

void i2c_master_init_bus()
{
    // Configure the i2c bus
    i2c_master_bus_handle_t bus_handle;
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    // Configure device
    i2c_master_dev_handle_t dev_handle;
    i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = MPU6050_ADDR,
    .scl_speed_hz = 100000, // 100kHz
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));

    // Write to imu to configure it
    // Configure power mode
    uint8_t write_buf[2] = {0x6B, 0x00}; // The packet must have an address followed by the data
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), -1));

    // Configure Gyroscope
    uint8_t write_buf[2] = {0x1B, 0x08}; // The packet must have an address followed by the data
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), -1));

    // Configure Accelerometer
    uint8_t write_buf[2] = {0x1C, 0x00}; // The packet must have an address followed by the data
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), -1));
    
    // Read 
    uint8_t write_buf[1] = 0x3B;
    uint8_t raw_imu[14];
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, write_buf, sizeof(write_buf), raw_imu, sizeof(raw_imu), -1));
    
    int16_t accel_x = (int16_t)((raw_imu[0] << 8) | raw_imu[1]);
    int16_t accel_y = (int16_t)((raw_imu[2] << 8) | raw_imu[3]);
    int16_t accel_z = (int16_t)((raw_imu[4] << 8) | raw_imu[5]);
    int16_t temp_raw = (int16_t)((raw_imu[6] << 8) | raw_imu[7]);
    int16_t gyro_x = (int16_t)((raw_imu[8] << 8) | raw_imu[9]);
    int16_t gyro_y = (int16_t)((raw_imu[10] << 8) | raw_imu[11]);
    int16_t gyro_z = (int16_t)((raw_imu[12] << 8) | raw_imu[13]);

    float ax = accel_x / 16384.0;
    float ay = accel_y / 16384.0;
    float az = accel_z / 16384.0;

    float gx = gyro_x / 65.5;
    float gy = gyro_y / 65.5;
    float gz = gyro_z / 65.5;















































}

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