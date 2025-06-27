#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "ultrasonic.h"

#define TRIG_GPIO 1
#define ECHO_GPIO 2
#define SOUND_SPEED 0.0343 // Sound speed = 34300 cm/s = 0.0343 cm/us

static const char *TAG = "ultrasonic";

void ultrasonic_init() {
    // Configure TRIG GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TRIG_GPIO), // which GPIO to configure
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Configure ECHO GPIO
    io_conf.pin_bit_mask = (1ULL << ECHO_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);

    // Clear TRIG
    gpio_set_level(TRIG_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
}

static float read_distance() {
    // Sets the TRIG on HIGH state for 10 micro seconds
    gpio_set_level(TRIG_GPIO, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_GPIO, 0);

    // Wait for echo to go high
    int64_t start_time = esp_timer_get_time();
    while (gpio_get_level(ECHO_GPIO) == 0) {
        if ((esp_timer_get_time() - start_time) > 100000) {
            ESP_LOGE(TAG, "ECHO setup timeout"); // Setup timeout in 100 ms
            return -1;
        }
    }

    int64_t echo_start = esp_timer_get_time();

    // Wait for echo to go low
    while (gpio_get_level(ECHO_GPIO) == 1) {
        if ((esp_timer_get_time() - echo_start) > 30000) {
            ESP_LOGI(TAG, "Nothing detected");
            return -1;
        }
    }

    int64_t echo_end = esp_timer_get_time();
    int64_t duration_us = echo_end - echo_start; 

    float distance_cm = (duration_us * SOUND_SPEED) / 2;
    return distance_cm;
}

void ultrasonic_task(void *param) {
    while (1) {
        float distance_cm = read_distance();
        if (distance_cm >= 0) {
            ESP_LOGI(TAG, "Disance: %.2f", distance_cm);
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}