#include <stdio.h>
#include <string.h>
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
#include "store/config/ble_store_config.h"
#include "xbox_ble.h"
#include "utils.h"
#include "esp_bt.h"

#define GATTS_SERVICE_UUID 0x00FF
#define SPEED_CHAR_UUID    0xFF01
#define STEER_CHAR_UUID    0xFF02

static const char *TAG = "phone_ble";
static uint8_t own_addr_type;

/* Characteristic value buffers */
static uint8_t speed_val[4] = "0";
static uint8_t direction_val[4] = "0";

/* GATT server definition */
static const struct ble_gatt_svc_def gatt_svcs[] {
    {
        /*** Service: Car Control ***/
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(GATTS_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* Characteristic: Speed */
                .uuid = BLE_UUID16_DECLARE(SPEED_CHAR_UUID),
                .access_cb = speed_control,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                /* Characteristic: Direction */
                .uuid = BLE_UUID16_DECLARE(STEER_CHAR_UUID),
                .access_cb = direction_control,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {0}, /* No more characteristics */
        },
    },
    {0}, /* No more services */  
};

static int speed_control(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            os_mbuf_append(ctxt->om, speed_val, strlen((char *)speed_val));
            return 0;
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            memset(speed_val, 0, sizeof(speed_val));
            os_mbuf_copydata(ctxt->om, 0, ctxt->om->om_len, speed_val);
            ESP_LOGI(TAG, "Received SPEED = %s", speed_val);
            return 0;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

static int direction_control(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            os_mbuf_append(ctxt->om, direction_val, strlen((char *)direction_val));
            return 0;
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            memset(direction_val, 0, sizeof(direction_val));
            os_mbuf_copydata(ctxt->om, 0, ctxt->om->om_len, direction_val);
            ESP_LOGI(TAG, "Received DIRECTION = %s", direction_val);
            return 0;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }    
}


static void on_stack_sync() {
    /* On stack sync, do advertising initialization */
    // on_stack_sync is called when host has synced with controller
    int rc = 0;
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "device does not have any available bt address!");
        return;
    }

    /* Figure out BT address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to infer address type, error code: %d", rc);
        return;
    }

    start_advertising();    
}


static void start_advertising(void) {
    int rc;
    
    // GAP - device name definition
    struct ble_hs_adv_fields adv_fields = {0};
    const char *device_name;

    /* Set advertising flags */
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /* Set device name */
    device_name = ble_svc_gap_device_name(); // Read the BLE device name
    adv_fields.name = (uint8_t *)device_name;
    adv_fields.name_len = strlen(device_name);
    adv_fields.name_is_complete = 1;

    /* Set advertiement fields */
    rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set advertising data, error code: %d", rc);
        return;
    }

    // GAP - device connectivity definition
    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable or non-connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discoverable or non-discoverable    
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to start advertising, error code: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "advertising started!");
}


/*
 * NimBLE applies an event-driven model to keep GAP service going
 * gap_event_handler is a callback function registered when calling
 * ble_gap_adv_start API and called when a GAP event arrives
 */
static int gap_event_handler(struct ble_gap_event *event, void *arg) {
    
}

void app_main() {
    esp_err_t ret;

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
    ble_svc_gap_device_name_set("ESP32-Server");

    /* GATT server initialization */
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs); // Update GATT services counter
    ble_gatts_add_svcs(gatt_svcs); // Add GATT services

    /* NimBLE host configuration initialization */
    /* Set host callbacks */
    ble_hs_cfg.sync_cb = on_stack_sync; // Called when BLE host syncs with controller
     
}