#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "utils.h"
#include "ina.h"
#include "driver/i2c_master.h"

// #define INA219_ESP_ADDR 0x40
#define INA219_LM_ADDR 0x40
#define INA219_RM_ADDR 0x40

static const char *TAG = "ina";

static QueueHandle_t uart_tx_queue;
static i2c_master_bus_handle_t bus_handle_0;
static i2c_master_bus_handle_t bus_handle_1;
static i2c_master_dev_handle_t lm_dev_handle;
static i2c_master_dev_handle_t rm_dev_handle;

void ina_init() {
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(I2C_NUM_0, &bus_handle_0));
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(I2C_NUM_1, &bus_handle_1));

    // Configure ina devices
    i2c_device_config_t dev_cfg_0;
    dev_cfg_0.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg_0.scl_speed_hz = 100000; // 100kHz
    dev_cfg_0.device_address = INA219_LM_ADDR;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle_0, &dev_cfg_0, &lm_dev_handle));

    i2c_device_config_t dev_cfg_1;
    dev_cfg_1.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg_1.scl_speed_hz = 100000; // 100kHz
    dev_cfg_1.device_address = INA219_RM_ADDR;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle_1, &dev_cfg_1, &rm_dev_handle));
}

static void configure(i2c_master_dev_handle_t dev_handle) {
    // Reset & Configure
    uint8_t write_buf[3] = {0x00, 0x39, 0x9F};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), -1));

    // Configure Calibration
    // Adafruit INA219 breakout board comes with a 0.1Ω shunt resistor and supports up to 3.2A current
    write_buf[0] = 0x05;
    write_buf[1] = 0x10;
    write_buf[2] = 0x00;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), -1));
}

static single_ina_t read_ina(i2c_master_dev_handle_t dev_handle) {
    uint8_t write_buf[1];
    uint8_t read_buf[2];    
    uint16_t value;

    write_buf[0] = 0x02;
    // Read bus voltage
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, write_buf, sizeof(write_buf), read_buf, sizeof(read_buf), -1));
    value = (read_buf[0] << 8) | read_buf[1];
    // Bit 3-15 contains actual data, so right shift by 3 bit.
    // Each bit represents 4 mV (LSB = 4 mV), so multiply the value by 4
    // Divide by 1000 to convert mV to V
    float bus_voltage = (float)((value >> 3) * 4) / 1000.0;  // in Volts

    // Read shunt voltage
    write_buf[0] = 0x01;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, write_buf, sizeof(write_buf), read_buf, sizeof(read_buf), -1));
    value = (read_buf[0] << 8) | read_buf[1];
    // convert unsign to sign
    // Each bit represents 10uV (LSB = 10uV), so multiply the value by 10
    // Divide by 1000 to convert uV to mV
    float shunt_voltage = (float)((int16_t)value * 10) / 1000;  // in mV

    // Read current
    // Ohm's law: I = V / R; R = 0.1Ω
    float current = shunt_voltage / 0.1;

    single_ina_t ina;
    ina.voltage = bus_voltage;
    ina.current = current;

    return ina;
}

void ina_task(void *param) {
    uart_tx_queue = (QueueHandle_t)param;
    
    configure(lm_dev_handle);
    configure(rm_dev_handle);

    while (1) {
        single_ina_t lm_ina = read_ina(lm_dev_handle);
        single_ina_t rm_ina = read_ina(rm_dev_handle);

        ESP_LOGI(TAG, "Left Voltage=%.2f V, Current=%.2f mA", lm_ina.voltage, lm_ina.current);
        ESP_LOGI(TAG, "Right Voltage=%.2f V, Current=%.2f mA", rm_ina.voltage, rm_ina.current);
        
        ina_input_t ina;
        ina.left_voltage = lm_ina.voltage;
        ina.left_current = lm_ina.current;
        ina.right_voltage = rm_ina.voltage;
        ina.right_current = rm_ina.current;

        // send ina data to UART
        uart_tx_message_t msg;
        msg.type = MSG_TYPE_INA;
        memcpy(msg.payload, &ina, sizeof(ina));

        if (xQueueSend(uart_tx_queue, &msg, portMAX_DELAY) != pdTRUE) {
            ESP_LOGI(TAG, "Queue full\n");
        }

        vTaskDelay(500 / portTICK_PERIOD_MS); // 500 ms
    }
}