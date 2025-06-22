#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "jetson_spi.h"

#define CS_GPIO   10
#define SCLK_GPIO 12
#define MOSI_GPIO 11
#define MISO_GPIO 13

const static char *TAG = "jetson_spi";

void jetson_spi_task(void *param) {
    esp_err_t ret;

    // SPI bus config
    spi_bus_config_t buscfg = {
        .mosi_io_num = MOSI_GPIO,
        .miso_io_num = MISO_GPIO,
        .sclk_io_num = SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    // SPI slave interface config
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = CS_GPIO,
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = NULL,
        .post_trans_cb = NULL,
    };

    // Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(MOSI_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SCLK_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(CS_GPIO, GPIO_PULLUP_ONLY);

    // Initialize SPI slave interface
    ret = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    // Buffers
    char *sendbuf = spi_bus_dma_memory_alloc(SPI2_HOST, 129, 0);
    char *recvbuf = spi_bus_dma_memory_alloc(SPI2_HOST, 129, 0);
    assert(sendbuf && recvbuf);

    spi_slave_transaction_t t = {0};

    while (1) {
        // Clear receive buffer, set send buffer
        memset(recvbuf, 0x00, 129);
        sprintf(sendbuf, "Dummy data from ESP.");

        // Set up a transaction of 128 bytes to send/receive
        t.length = 128 * 8; // in bits
        t.tx_buffer = sendbuf;
        t.rx_buffer = recvbuf;

        /* This call enables the SPI slave interface to send/receive to the sendbuf and recvbuf. The transaction is
        initialized by the SPI master, so it will not actually happen until the master starts a hardware transaction
        by pulling CS low and pulsing the clock etc.
        */
        ESP_LOGI(TAG, "Waiting for SPI transaction...");
        ret = spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);

        // spi_slave_transmit does not return until the master has done a transmission, so by here we have sent our data and
        // received data from the master. Print it.
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Received: %s\n", recvbuf);
        }
 
        vTaskDelay(100);
    }
}
