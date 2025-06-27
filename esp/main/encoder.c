#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "encoder.h"

#define ENCODER_L_GPIO 36
#define ENCODER_R_GPIO 37
#define PULSES_PER_REV 20
#define SPEED_CALC_INTERVAL_MS 1000  // How often to calculate speed

static const char *TAG = "encoder";
static volatile int pulse_count_l = 0;
static volatile int pulse_count_r = 0;
static uint64_t prev_time = 0;
static volatile uint64_t last_l_us = 0;
static volatile uint64_t last_r_us = 0;

// ISR: Called on each encoder pulse
static void IRAM_ATTR encoder_l_isr_handler(void* arg) {
    uint64_t now = esp_timer_get_time();
    if (now - last_l_us > 5000 && gpio_get_level(ENCODER_L_GPIO)) {
        pulse_count_l++;
        last_l_us = now;
    }
}

static void IRAM_ATTR encoder_r_isr_handler(void* arg) {
    uint64_t now = esp_timer_get_time();
    if (now - last_r_us > 5000 && gpio_get_level(ENCODER_R_GPIO)) {
        pulse_count_r++;
        last_r_us = now;
    }
}

void encoder_init() {
    // Configure encoder GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ENCODER_L_GPIO), // which GPIO to configure
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE, // Trigger an interrupt on the rising edge
    };
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL << ENCODER_R_GPIO);
    gpio_config(&io_conf);

    // Install GPIO ISR service and attach handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENCODER_L_GPIO, encoder_l_isr_handler, NULL);
    gpio_isr_handler_add(ENCODER_R_GPIO, encoder_r_isr_handler, NULL);

    // Record initial time
    prev_time = esp_timer_get_time();
}


void encoder_task(void *args) {
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(SPEED_CALC_INTERVAL_MS));

        uint64_t curr_time = esp_timer_get_time();
        uint64_t elapsed_time = curr_time - prev_time;
        prev_time = curr_time;

        int count_l, count_r;
        vPortEnterCritical(&mux);
        count_l = pulse_count_l;
        count_r = pulse_count_r;
        pulse_count_l = 0;
        pulse_count_r = 0;
        vPortExitCritical(&mux);

        ESP_LOGI(TAG, "count_l: %d", count_l);

        float seconds = elapsed_time / 1e6;
        float rps_l = (count_l / (float)PULSES_PER_REV) / seconds;
        float rpm_l = rps_l * 60.0;
        float rps_r = (count_r / (float)PULSES_PER_REV) / seconds;
        float rpm_r = rps_r * 60.0;

        ESP_LOGI(TAG, "Left RPM: %.2f", rpm_l);
        ESP_LOGI(TAG, "Right RPM: %.2f", rpm_r);
    }
}
