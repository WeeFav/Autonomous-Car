#ifndef XBOX_BLE_H
#define XBOX_BLE_H

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
#include "store/config/ble_store_config.h"
#include "utils.h"

// Xbox Wireless Controller Service UUIDs
#define XBOX_SERVICE_UUID           0x1812  // HID Service
#define XBOX_REPORT_UUID            0x2A4D  // Report Characteristic
#define XBOX_REPORT_MAP_UUID        0x2A4B  // Report Map
#define XBOX_HID_INFO_UUID          0x2A4A  // HID Information
#define XBOX_CONTROL_POINT_UUID     0x2A4C  // HID Control Point
#define XBOX_CCCD_UUID              0x2902  // CCCD

/* Function declaration */
void start_scan(void);
int gap_event_handler(struct ble_gap_event *event, void *arg);
void ble_store_config_init(void);
int disc_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *service, void *arg);
int disc_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg);
int disc_desc_cb(uint16_t conn_handle, const struct ble_gatt_error *error, uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg);
// int report_map_read_cb(uint16_t conn_handle, const struct ble_gatt_error *error, struct ble_gatt_attr *attr, void *arg);
int notify_event_cb(uint16_t conn_handle, const struct ble_gatt_error *error, struct ble_gatt_attr *attr, void *arg);
void xbox_ble_task(void *param);

#endif