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

// Xbox Wireless Controller Service UUIDs
#define XBOX_SERVICE_UUID           0x1812  // HID Service
#define XBOX_REPORT_UUID            0x2A4D  // Report Characteristic
#define XBOX_REPORT_MAP_UUID        0x2A4B  // Report Map
#define XBOX_HID_INFO_UUID          0x2A4A  // HID Information
#define XBOX_CONTROL_POINT_UUID     0x2A4C  // HID Control Point
#define XBOX_CCCD_UUID              0x2902  // CCCD

static const char *TAG = "xbox_ble";
static uint16_t conn_handle;
static uint16_t end_handle = 32;
static xbox_input_t prev = {0};
static xbox_ble_task_args_t *xbox_ble_task_args;

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
    disc_params.passive = 0;
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
    int rc;
    struct ble_hs_adv_fields fields;
    char name[BLE_HS_ADV_MAX_SZ + 1] = {0};
    uint8_t own_addr_type;
    struct ble_gap_conn_desc desc;
    // char output[512];

    if (event->type != 12) {
        ESP_LOGI(TAG, "GAP Event: %d", event->type);
    }

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

            if (strstr(name, "Xbox Wireless Controller") != NULL) {
                /* Scanning must be stopped before a connection can be initiated. */
                rc = ble_gap_disc_cancel();
                if (rc != 0) {
                    ESP_LOGE(TAG, "Failed to cancel scan; rc=%d\n", rc);
                    return 0;
                }

                rc = ble_hs_id_infer_auto(0, &own_addr_type);
                if (rc != 0) {
                    ESP_LOGE(TAG, "failed to infer address type, error code: %d", rc);
                    return 0;
                }
                /* Try to connect the the advertiser.  Allow 30 seconds (30000 ms) for timeout. */
                rc = ble_gap_connect(own_addr_type, &event->disc.addr, 30000, NULL, gap_event_handler, NULL);
                if (rc != 0) {
                    ESP_LOGE(TAG, "Error: Failed to connect to device, error code: %d", rc);
                    return 0;
                }
            }
            break;

        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(TAG, "Connection established");
                conn_handle = event->connect.conn_handle;

                // Make sure connection is found
                rc = ble_gap_conn_find(conn_handle, &desc);
                assert(rc == 0);

                /** Initiate security - It will perform
                    * Pairing (Exchange keys)
                    * Bonding (Store keys)
                    * Encryption (Enable encryption)
                    * Will invoke event BLE_GAP_EVENT_ENC_CHANGE
                **/
                rc = ble_gap_security_initiate(conn_handle);
                if (rc != 0) {
                    ESP_LOGE(TAG, "Security could not be initiated, rc = %d", rc);
                    return ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                } 

            } 
            else {
                /* Connection attempt failed; resume scanning. */
                ESP_LOGE(TAG, "Error: Connection failed; status=%d\n", event->connect.status);
                start_scan();
            }

            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Disconnected. (reason: %d)", event->disconnect.reason);
            start_scan();
            break;

        case BLE_GAP_EVENT_ENC_CHANGE:
            ESP_LOGI(TAG, "Encryption change; status=%d", event->enc_change.status);

            if (event->enc_change.status == 0) {
                // Make sure connection is found after encryption
                rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
                assert(rc == 0);

                /* Perform service discovery after encryption has been successfully enabled */
                ble_gattc_disc_all_svcs(conn_handle, disc_svc_cb, NULL);
            }
            else {
                ESP_LOGE(TAG, "The encryption state change attempt failed; status=%d\n", event->enc_change.status);
                ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                start_scan();
            }

            break;

        case BLE_GAP_EVENT_NOTIFY_RX:
            // Process only if the notification comes from xbox HID
            if (event->notify_rx.attr_handle == 30) {
                xbox_input_t* report = (xbox_input_t*)event->notify_rx.om->om_data;
                
                if (compare_xbox_report(&prev, report)) {
                    // format_xbox_report(output, report);
                    // ESP_LOGI(TAG, "Report: \n%s", output);

                    // Send to motor
                    if (xQueueSend(xbox_ble_task_args->xbox_input_queue, (void *)report, 0) != pdTRUE) {
                        ESP_LOGI(TAG, "Queue full\n");
                    }

                    // // Send to UART
                    // uart_tx_message_t msg;
                    // msg.type = MSG_TYPE_XBOX;
                    // msg.size = sizeof(*report);
                    // memcpy(msg.payload, report, sizeof(*report));

                    // if (xQueueSend(xbox_ble_task_args->uart_tx_queue, &msg, portMAX_DELAY) != pdTRUE) {
                    //     ESP_LOGI(TAG, "Queue full\n");
                    // }
                    // ESP_LOGI(TAG, "xbox controller input sent to queue");

                    prev = *report;
                }
            }

            // os_mbuf_free_chain(event->notify_rx.om);

            break;

        case BLE_GAP_EVENT_REPEAT_PAIRING:
            /* We already have a bond with the peer, but it is attempting to
            * establish a new secure link.  This app sacrifices security for
            * convenience: just throw away the old bond and accept the new link.
            */
            ESP_LOGI(TAG, "Repeat pairing");

            /* Delete the old bond. */
            rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
            assert(rc == 0);
            ble_store_util_delete_peer(&desc.peer_id_addr);

            /* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should
            * continue with the pairing operation.
            */
            return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    return 0;
}

int disc_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *service, void *arg) {
    switch (error->status) {
        case 0:
            ESP_LOGI(TAG, "Service discovered: UUID=0x%04x, start_handle=%d, end_handle=%d", 
                    ble_uuid_u16(&service->uuid.u), service->start_handle, service->end_handle);
            
            // If this is the HID service, discover characteristics
            if (ble_uuid_u16(&service->uuid.u) == XBOX_SERVICE_UUID) {
                ESP_LOGI(TAG, "Found HID service, discovering characteristics...");
                ble_gattc_disc_all_chrs(conn_handle, service->start_handle, service->end_handle, disc_chr_cb, NULL);
            }

            break;

        case BLE_HS_EDONE:
            ESP_LOGI(TAG, "All services discovered");
            break;

        default:
            ESP_LOGE(TAG, "Service discovery failed: %d", error->status);
            break;
    }

    return 0;
}

int disc_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg) {
    switch (error->status) {
        case 0:
            ESP_LOGI(TAG, "Characteristic discovered: UUID=0x%04x, handle=%d, value_handle=%d, properties=0x%X", 
                    ble_uuid_u16(&chr->uuid.u), chr->def_handle, chr->val_handle, chr->properties);

            // WE DON'T NEED TO READ REPORT MAP BECAUSE IT ONLY GIVES PARTIAL MAPPING. WE CAN FIND FULL MAPPING ONLINE.
            // Read Report Map
            /*
            if (ble_uuid_u16(&chr->uuid.u) == XBOX_REPORT_MAP_UUID) {
                ble_gattc_read(conn_handle, chr->val_handle, report_map_read_cb, NULL);
            }
            */

            /*
            If the characteristic uuid is 0x2a4d and has the notify property, discover its
            descriptors to find CCCD. We need to write to CCCD to enable notifications.
            */ 
            if ((ble_uuid_u16(&chr->uuid.u) == XBOX_REPORT_UUID) && (chr->properties & BLE_GATT_CHR_PROP_NOTIFY)) {
                ble_gattc_disc_all_dscs(conn_handle, chr->def_handle, end_handle, disc_desc_cb, NULL);
            }

            break;
        case BLE_HS_EDONE:
            ESP_LOGI(TAG, "All characteristics discovered");
            break;
        default:
            ESP_LOGE(TAG, "Characteristic discovery failed: %d", error->status);
            break;
    }
    
    return 0;
}

int disc_desc_cb(uint16_t conn_handle, const struct ble_gatt_error *error, uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg) {
    int rc;
    uint8_t value[2];

    switch (error->status) {
        case 0:
            // Look for Client Characteristic Configuration Descriptor (CCCD) - UUID 0x2902
            if ((ble_uuid_u16(&dsc->uuid.u) == XBOX_CCCD_UUID)) {
                ESP_LOGI(TAG, "Found CCCD (handle: %d)", dsc->handle);

                // Subscribe to notifications by writing to the CCCD
                value[0] = 1;
                value[1] = 0;
                rc = ble_gattc_write_flat(conn_handle, dsc->handle, value, sizeof(value), notify_event_cb, NULL);
                if (rc != 0) {
                    ESP_LOGE(TAG, "Failed to subscribe to notifications: %d", rc);
                }
               
            }
            break;

        case BLE_HS_EDONE:
            ESP_LOGI(TAG, "All descriptors discovered");
            break;

        default:
            ESP_LOGE(TAG, "Descriptors discovery failed: %d", error->status);
            break;
    }

    return 0;
}

int notify_event_cb(uint16_t conn_handle, const struct ble_gatt_error *error, struct ble_gatt_attr *attr, void *arg) {
    if (error->status == 0) {
        ESP_LOGI(TAG, "Successfully subscribed to notifications");
    } else {
        ESP_LOGE(TAG, "Failed to subscribe to notifications: %d", error->status);
    }
    return 0;    
}

/*
int report_map_read_cb(uint16_t conn_handle, const struct ble_gatt_error *error, struct ble_gatt_attr *attr, void *arg) {
    if (error->status == 0 && attr != NULL) {
        ESP_LOGI(TAG, "Report Map length: %d", attr->om->om_len);
        
        uint8_t *data = attr->om->om_data;
        for (int i = 0; i < attr->om->om_len; i++) {
            printf("%02X ", data[i]);
        }
        printf("\n");
    }
    else {
        ESP_LOGE(TAG, "Failed to read Report Map: %d", error->status);

    }

    return 0;
}
*/

void xbox_ble_init() {
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
        ESP_LOGE(TAG, "failed to initialize nimble stack, error code: %d ", ret);
        return;
    }

    /* GAP service initialization */
    ble_svc_gap_init();

    /* Set GAP device name */
    int rc = 0;
    rc = ble_svc_gap_device_name_set("ESP32-Xbox-Client");
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set device name to %s, error code: %d", "ESP32-Xbox-Client", rc);
        return;
    }

    /* NimBLE host configuration initialization */
    /* Set host callbacks */
    ble_hs_cfg.reset_cb = on_stack_reset; // Called when BLE host resets
    ble_hs_cfg.sync_cb = on_stack_sync; // Called when BLE host syncs with controller
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr; // For BLE store operations

    // --- Configure BLE security properties ---
    ble_hs_cfg.sm_io_cap = 0;  // Typically for Just Works
    ble_hs_cfg.sm_bonding = 1; // Enable bonding (store keys for future reconnections)
    ble_hs_cfg.sm_mitm = 0;    // No Man-In-The-Middle protection (for Just Works)
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID; // Distribute Encryption and Identity keys
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID; // Accept Encryption and Identity keys from peer

    /* Store host configuration */
    ble_store_config_init();

    // Max TX power
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
}

void xbox_ble_task(void *param) {
    xbox_ble_task_args = (xbox_ble_task_args_t *)param;

    // Start the NimBLE host task (this handles the BLE events)
    // Creates a new FreeRTOS task to run the BLE host
    nimble_port_freertos_init(ble_host_task);

    free(xbox_ble_task_args);
    vTaskDelete(NULL); // Task no longer needed, NimBLE runs its own task
}