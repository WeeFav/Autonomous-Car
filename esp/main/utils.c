#include "xbox_ble.h"
#include "utils.h"

void format_xbox_report(char *output, const xbox_report_payload_t *report) {
    sprintf(output,
        "Left Stick: X=%u Y=%u | Right Stick: X=%u Y=%u\n"
        "Left Trigger: %u | Right Trigger: %u\n"
        "D-Pad: %u\n"
        "Buttons: A=%u B=%u X=%u Y=%u LB=%u RB=%u\n"
        "Stick Presses: LS=%u RS=%u\n"
        "Menu Buttons: View=%u Menu=%u Share=%u\n"
        "RFU Bits: %u %u %u %u %u %u\n",
        report->left_stick_x,
        report->left_stick_y,
        report->right_stick_x,
        report->right_stick_y,
        report->left_trigger,
        report->right_trigger,
        report->dpad,
        report->button_a,
        report->button_b,
        report->button_x,
        report->button_y,
        report->left_bumper,
        report->right_bumper,
        report->left_stick_press,
        report->right_stick_press,
        report->view_button,
        report->menu_button,
        report->share_button,
        report->rfu_1,
        report->rfu_2,
        report->rfu_3,
        report->rfu_4,
        report->rfu_5,
        report->rfu_6
    );
}

bool compare_xbox_report(const xbox_report_payload_t *prev, const xbox_report_payload_t *curr) {
    // Ignore left and right stick
    
    // Compare triggers
    if (prev->left_trigger != curr->left_trigger ||
        prev->right_trigger != curr->right_trigger)
        return true;

    // Compare D-pad
    if (prev->dpad != curr->dpad)
        return true;

    // Compare face buttons and bumpers
    if (prev->button_a != curr->button_a ||
        prev->button_b != curr->button_b ||
        prev->button_x != curr->button_x ||
        prev->button_y != curr->button_y ||
        prev->left_bumper != curr->left_bumper ||
        prev->right_bumper != curr->right_bumper)
        return true;

    // Compare stick presses
    if (prev->left_stick_press != curr->left_stick_press ||
        prev->right_stick_press != curr->right_stick_press)
        return true;

    // Compare menu/view/share buttons
    if (prev->view_button != curr->view_button ||
        prev->menu_button != curr->menu_button ||
        prev->share_button != curr->share_button)
        return true;

    /*
    if (prev->rfu_1 != curr->rfu_1 ||
        prev->rfu_2 != curr->rfu_2 ||
        prev->rfu_3 != curr->rfu_3 ||
        prev->rfu_4 != curr->rfu_4 ||
        prev->rfu_5 != curr->rfu_5 ||
        prev->rfu_6 != curr->rfu_6)
        return true;
    */

    // If all relevant fields are equal
    return false;
}