/**
 * @file motor_control.c
 * @brief Implementation of the motor state machine and control loop.
 *
 * This module manages the high-level state machine for the motor controller node.
 * It is responsible for orchestrating system startup, enforcing safety interlocks,
 * and transitioning between operational states based on telemetry data fetched
 * from the CAN bus. It acts as the primary decision-making hub, ensuring that
 * motor actuation (PWM) only occurs when the system is in a valid, safe, and
 * armed state.
 */

#include "motor_control.h"
#include "motor_can.h"
#include "motor_pwm.h"
#include "motor_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "MOTOR_CTRL";

typedef enum {
    STATE_INIT,
    STATE_IDLE,
    STATE_RUNNING,
    STATE_FAULT
} motor_state_t;

static motor_state_t current_state = STATE_INIT;

void ts_motorControlLoop(void *arg) {
    // Phase 0: System Initialization
    motor_pwm_init();
    motor_can_init(); // Spawns the background RX wrapper

    current_state = STATE_IDLE;
    ESP_LOGI(TAG, "Motor Control Loop Started.");

    while (1) {
        bool throttle_fault = false;

        // Phase 1: Read Network Inputs
        // Instantly fetches from the thread-safe wrapper cache
        bool is_timeout = motor_can_is_timeout();
        bool is_temp_high = motor_can_is_temp_high();
        uint8_t throttle = motor_can_get_throttle(&throttle_fault);

        // Phase 2: Evaluate State Machine
        // Safety always overrides operational states
        if (is_timeout || is_temp_high || throttle_fault) {
            if (current_state != STATE_FAULT) {
                ESP_LOGE(TAG, "CRITICAL: Entering FAULT state.");
            }
            current_state = STATE_FAULT; // Lock system into safe mode
        } 
        else if (current_state == FAULT) {
            // @TODO: Add recovery logic (e.g., waiting for zero throttle)
        }
        else if (throttle > 0) {
            current_state = STATE_RUNNING;
        } 
        else {
            current_state = STATE_IDLE;
        }

        // Phase 3: Actuate Hardware
        switch (current_state) {
            case STATE_FAULT:
            case STATE_IDLE:
                motor_pwm_set_throttle(0); // Hardware safe-stop
                break;

            case STATE_RUNNING:
                motor_pwm_set_throttle(throttle);
                break;

            default:
                motor_pwm_set_throttle(0); // Fallback safe-stop
                break;
        }

        // Yield CPU for the control loop period
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
