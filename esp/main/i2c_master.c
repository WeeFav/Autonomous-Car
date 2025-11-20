#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c_master.h"

#define SDA_GPIO_0 8
#define SCL_GPIO_0 9
#define SDA_GPIO_1 10
#define SCL_GPIO_1 11

static const char *TAG = "i2c_master";

void i2c_master_init() {
    // Configure the i2c bus
    i2c_master_bus_handle_t bus_handle_0;
    i2c_master_bus_config_t bus_config_0 = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = SDA_GPIO_0,
        .scl_io_num = SCL_GPIO_0,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config_0, &bus_handle_0));

    i2c_master_bus_handle_t bus_handle_1;
    i2c_master_bus_config_t bus_config_1 = {
        .i2c_port = I2C_NUM_1,
        .sda_io_num = SDA_GPIO_1,
        .scl_io_num = SCL_GPIO_1,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config_1, &bus_handle_1));

    // ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    // ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
}