#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "pid.h"
#include "xbox_ble.h"
#include "encoder.h"
#include "utils.h"
#include "freertos/semphr.h"

static const char *TAG = "PID";

static float kp = 0.5f;
static float ki = 0.2f;
static float kd = 0.05f;
static float prev_error = 0.0f;
static float integral = 0.0f;
static float output_min = 0.0f;
static float output_max = 100.0f;
static float dt = 0.01f;

static float target_speed = 0;
static float error = 0;

static float pid_compute(float setpoint_pwm, float measured_speed) {
    // Map setpoint PWM (0–100) to target speed (depends on your motor)
    // Example: assume 100 PWM ≈ 300 RPM
    target_speed = (setpoint_pwm / 100.0f) * 250.0f;  

    error = target_speed - measured_speed;
    if (fabs(error) < 0.01) {
        integral = 0;
    }
    else {
        integral += error * dt;
    }
    float derivative = (error - prev_error) / dt;

    // integral = 0;
    derivative = 0;

    float output = kp * error + ki * integral + kd * derivative;

    // Clamp and anti-windup
    if (output > output_max) {
        output = output_max;
        integral -= error * dt;
    } else if (output < output_min) {
        output = output_min;
        integral -= error * dt;
    }

    prev_error = error;

    return output; // Final PWM duty cycle (0–100)
}

void pid_task() {
    motor_input_t input;
    float measured_speed = 0;

    while (1) {
        if (xSemaphoreTake(pid_xbox_mutex, portMAX_DELAY) == pdTRUE)
        {
            input = pid_xbox_input;
            xSemaphoreGive(pid_xbox_mutex);
        }
        float setpoint_pwm = input.pwm; // Desired "speed" command (0–100)
        
        if (xSemaphoreTake(pid_encoder_mutex, portMAX_DELAY) == pdTRUE)
        {
            measured_speed = pid_encoder_input;
            xSemaphoreGive(pid_encoder_mutex);
        }        
        float control_pwm = pid_compute(setpoint_pwm, measured_speed);

        float final_output = setpoint_pwm + control_pwm;

        // Clamp
        if (final_output > output_max) {
            final_output = output_max;
        } else if (final_output < output_min) {
            final_output = output_min;
        }

        input.pwm = (uint8_t) final_output;
        if (xQueueSend(motor_queue, &input, portMAX_DELAY) != pdTRUE) {
            ESP_LOGI(TAG, "Queue full\n");
        }        
        
        ESP_LOGI(TAG, "Cmd: %.1f | TS: %.1f | Meas: %.1f RPM | Error: %.1f |PWM Out: %.1f | Final: %.1f ", setpoint_pwm, target_speed, measured_speed, error, control_pwm, final_output);
        
        vTaskDelay(pdMS_TO_TICKS((int)(dt * 1000))); // 10 ms loop time, 100 Hz
    }
}
