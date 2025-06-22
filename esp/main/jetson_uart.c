#include "driver/uart.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "jetson_uart.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "utils.h"

#define TX_GPIO 17
#define RX_GPIO 18
#define UART_BUFFER_SIZE (1024 * 2)

const static char *TAG = "jetson_uart";

static QueueHandle_t uart_event_queue;

void uart_init() {
    const uart_port_t uart_num = UART_NUM_1;
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Set GPIO pin to UART
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, TX_GPIO, RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    /*
    There are 2 RX buffer: the hardware RX FIFO buffer (128 bytes) and software RX ring buffer. Data from
    UART gets read into hardware RX FIFO buffer, then copied into software RX ring buffer automatically.
    uart_read_bytes() read from software RX ring buffer.

    When called uart_write_bytes(), the data is first queued in the software TX ring buffer.
    When there is free space in the hardware TX FIFO buffer, an interrupt service routine (ISR) moves 
    the data from the TX ring buffer to the TX FIFO buffer in the background.
    */
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 10, &uart_event_queue, 0));
}

void jetson_uart_task(void *param) {
    QueueHandle_t uart_tx_queue = (QueueHandle_t)param;
    uart_event_t event;
    uint8_t rx_data[UART_BUFFER_SIZE];

    while (1) {
        // Wait for UART event
        if (xQueueReceive(uart_event_queue, &event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    uart_read_bytes(UART_NUM_1, rx_data, event.size, portMAX_DELAY);
                    rx_data[event.size] = '\0';
                    ESP_LOGI(TAG, "Received: %s", rx_data);
                    break;

                case UART_FIFO_OVF:
                case UART_BUFFER_FULL:
                    ESP_LOGE(TAG, "Overflow or buffer full, flushing input");
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart_event_queue);
                    break;

                default:
                    break;
            }
        }

        // // Handle outgoing messages from TX queue
        // uart_tx_message_t msg;
        // if (xQueueReceive(uart_tx_queue, &msg, portMAX_DELAY) == pdTRUE) {
        //     uart_write_bytes(UART_NUM_1, (const char *)&msg.type, sizeof(msg.type));
        //     uart_write_bytes(UART_NUM_1, (const char *)&msg.size, sizeof(msg.size));
        //     uart_write_bytes(UART_NUM_1, (const char *)msg.payload, msg.size);
        // }
    }
}