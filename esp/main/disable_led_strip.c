#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "disable_led_strip.h"
#include "sdkconfig.h"
#include "led_strip.h"

static led_strip_handle_t led_strip;
static const char *TAG = "disable_led_strip";

void disable_led(void)
{
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = 38,
        .max_leds = 1, // at least one LED on board
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    /* Set all LED off to clear all pixels */
    ESP_LOGI(TAG, "Turning the LED OFF");
    led_strip_clear(led_strip);
}