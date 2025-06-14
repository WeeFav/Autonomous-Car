#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "sdkconfig.h"
#include "esp_central.h"
#include "store/config/ble_store_config.h"

static const char *TAG = "xbox_ble";

void start_scan(void);
int gap_event_handler(struct ble_gap_event *event, void *arg);
void ble_store_config_init(void);

void on_stack_reset(int reason) {
    /* On reset, print reset reason to console */
    // on_stack_reset is called when host resets BLE stack due to errors
    ESP_LOGI(TAG, "nimble stack reset, reset reason: %d", reason);
}

void on_stack_sync(void) {
    /* On stack sync, do advertising initialization */
    // on_stack_sync is called when host has synced with controller
    start_scan();
}

void ble_host_task(void *param) {
    ESP_LOGI(TAG, "BLE Host Task started");
    nimble_port_run(); // starts the BLE host stack, blocks and runs indefinitely
    nimble_port_freertos_deinit();
}

// Start BLE scan
void start_scan(void) {
    int rc;
    uint8_t own_addr_type;

    /* Figure out BT address to use while advertising */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to infer address type, error code: %d", rc);
        return;
    }

    /* Configure and start the GAP discovery process */
    struct ble_gap_disc_params disc_params;
    disc_params.filter_duplicates = 1;
    disc_params.passive = 1;
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params, gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start discovery; rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "BLE scan started...");
    }
}

/*
NimBLE applies an event-driven model to keep GAP service going
gap_event_handler is a callback function registered when calling
ble_gap_disc API and called when a GAP event arrives
*/
int gap_event_handler(struct ble_gap_event *event, void *arg) {
    struct ble_hs_adv_fields fields;
    char name[BLE_HS_ADV_MAX_SZ + 1] = {0};

    /* Handle different GAP event */
    switch (event->type) {
        /* An advertisement report was received during GAP discovery. */
        case BLE_GAP_EVENT_DISC:
            ESP_LOGI(TAG, "found something...");
            ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);

            if (fields.name_len > 0) {
                memcpy(name, fields.name, fields.name_len);
                name[fields.name_len] = '\0';
                ESP_LOGI(TAG, "found %s", name);
            }
    }
    return 0;
}

void app_main(void) {
    esp_err_t ret;

    /*
    NVS (Non-Volatile Storage) flash initialization
    Dependency of BLE stack to store configurations
    */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize nvs flash, error code: %d ", ret);
        return;
    }

    /* NimBLE stack initialization */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize nimble stack, error code: %d ",
                 ret);
        return;
    }

    /* GAP service initialization */
    ble_svc_gap_init();

    /* Set GAP device name */
    int rc = 0;
    rc = ble_svc_gap_device_name_set("ESP32-Xbox-Client");
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set device name to %s, error code: %d",
                 "ESP32-Xbox-Client", rc);
        return;
    }

    /* NimBLE host configuration initialization */
    /* Set host callbacks */
    ble_hs_cfg.reset_cb = on_stack_reset; // Called when BLE host resets
    ble_hs_cfg.sync_cb = on_stack_sync; // Called when BLE host syncs with controller
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr; // For BLE store operations

    /* Store host configuration */
    ble_store_config_init();

    // Start the NimBLE host task (this handles the BLE events)
    // Creates a new FreeRTOS task to run the BLE host
    nimble_port_freertos_init(ble_host_task);
}