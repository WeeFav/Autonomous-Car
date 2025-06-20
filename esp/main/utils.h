#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

} xbox_report_payload_t;

void format_xbox_report(char *output, const xbox_report_payload_t *report);
bool compare_xbox_report(const xbox_report_payload_t *prev, const xbox_report_payload_t *curr);

#endif
