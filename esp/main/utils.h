#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// This struct represents the 16-byte payload of the controller report,
// excluding the Report ID.
typedef struct __attribute__((packed)) {
    // NOTE: The reportId field is REMOVED.

    // Joysticks (16-bit values, 0-65535)
    uint16_t left_stick_x;
    uint16_t left_stick_y;
    uint16_t right_stick_x;
    uint16_t right_stick_y;

    // Triggers (10-bit values, 0-1023)
    uint16_t left_trigger : 10;
    uint16_t left_trigger_padding : 6;
    uint16_t right_trigger : 10;
    uint16_t right_trigger_padding : 6;

    // D-Pad (4-bit value)
    uint8_t dpad : 4;
    uint8_t dpad_padding : 4;

    // Face Buttons & Bumpers (1-bit flags)
    uint8_t button_a : 1;
    uint8_t button_b : 1;
    uint8_t rfu_1 : 1;
    uint8_t button_x : 1;
    uint8_t button_y : 1;
    uint8_t rfu_2 : 1;
    uint8_t left_bumper : 1;
    uint8_t right_bumper : 1;

    // Special Buttons (1-bit flags)
    uint8_t rfu_3 : 1;
    uint8_t rfu_4 : 1;
    uint8_t view_button : 1;
    uint8_t menu_button : 1;
    uint8_t left_stick_press : 1;
    uint8_t right_stick_press : 1;
    uint8_t rfu_5 : 1;
    uint8_t rfu_6 : 1;

    // Share Button (1-bit flag)
    uint8_t share_button : 1;
    uint8_t share_button_padding : 7;

} xbox_input_t;

typedef struct {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
} imu_input_t;

typedef struct {
    float voltage, current;
} ina_input_t;

typedef enum {
    MSG_TYPE_XBOX,
    MSG_TYPE_IMU,
    MSG_TYPE_INA
} MessageType;

typedef struct {
    MessageType type;
    uint16_t size;
    uint8_t payload[24]; // maximum struct size is imu_input_t (24 bytes)
} uart_tx_message_t;

typedef struct {
    QueueHandle_t xbox_input_queue;
    QueueHandle_t uart_tx_queue;
} xbox_ble_task_args_t;

void format_xbox_report(char *output, const xbox_input_t *report);
bool compare_xbox_report(const xbox_input_t *prev, const xbox_input_t *curr);
void disable_led(void);

#endif
