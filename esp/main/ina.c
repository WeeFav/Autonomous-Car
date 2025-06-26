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

#define INA219_ESP_ADDR 0x40
#define INA219_LM_ADDR 0x41
#define INA219_RM_ADDR 0x44

static const char *TAG = "ina";

static QueueHandle_t uart_tx_queue;
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t esp_dev_handle;
static i2c_master_dev_handle_t lm_dev_handle;
static i2c_master_dev_handle_t rm_dev_handle;

void ina_init() {
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(I2C_NUM_0, &bus_handle));

    i2c_device_config_t dev_cfg;

    // Configure ina devices
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = INA219_ESP_ADDR;
    dev_cfg.scl_speed_hz = 100000; // 100kHz
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &esp_dev_handle));

    dev_cfg.device_address = INA219_LM_ADDR;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &lm_dev_handle));

    dev_cfg.device_address = INA219_RM_ADDR;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &rm_dev_handle));
}

void configure(i2c_master_dev_handle_t dev_handle) {
    // Reset & Configure
    uint8_t write_buf[3] = {0x00, 0x39, 0x9F};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), -1));

    // Configure Calibration
    // Adafruit INA219 breakout board comes with a 0.1Ω shunt resistor and supports up to 3.2A current
    uint8_t write_buf[3] = {0x05, 0x10, 0x00};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), -1));
}

ina_input_t read_ina() {
    uint8_t write_buf[1];
    uint8_t read_buf[2];    
    uint16_t value;

    write_buf[1] = 0x02;
    // Read bus voltage
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, write_buf, sizeof(write_buf), read_buf, sizeof(read_buf), -1));
    value = (read_buf[0] << 8) | read_buf[1];
    // Bit 3-15 contains actual data, so right shift by 3 bit.
    // Each bit represents 4 mV (LSB = 4 mV), so multiply the value by 4
    // Divide by 1000 to convert mV to V
    float bus_voltage = (float)((value >> 3) * 4) / 1000.0;  // in Volts

    // Read shunt voltage
    write_buf[1] = 0x01;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, write_buf, sizeof(write_buf), read_buf, sizeof(read_buf), -1));
    value = (read_buf[0] << 8) | read_buf[1];
    // convert unsign to sign
    // Each bit represents 10uV (LSB = 10uV), so multiply the value by 10
    // Divide by 1000 to convert uV to mV
    float shunt_voltage = (float)((int16_t)value * 10) / 1000;  // in mV

    // Read current
    // Ohm's law: I = V / R; R = 0.1Ω
    float current = shunt_voltage / 0.1;

    ina_input_t ina;
    ina.voltage = bus_voltage;
    ina.current = current;
    return ina;
}

void send_uart(ina_input_t ina) {
    uart_tx_message_t msg;
    // send ina data to UART
    msg.type = MSG_TYPE_INA;
    msg.size = sizeof(ina);
    memcpy(msg.payload, &ina, sizeof(ina));

    if (xQueueSend(uart_tx_queue, &msg, portMAX_DELAY) != pdTRUE) {
        ESP_LOGI(TAG, "Queue full\n");
    }
}

void ina_task(void *param) {
    uart_tx_queue = (QueueHandle_t)param;
    
    configure(esp_dev_handle);
    configure(lm_dev_handle);
    configure(rm_dev_handle);

    while (1) {
        ina_input_t esp_ina = read_ina();
        ina_input_t lm_ina = read_ina();
        ina_input_t rm_ina = read_ina();

        send_uart(esp_ina);
        send_uart(lm_ina);
        send_uart(rm_ina);

        vTaskDelay(500 / portTICK_PERIOD_MS); // 500 ms
    }
}