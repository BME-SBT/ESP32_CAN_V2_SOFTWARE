/**
 * @file motor_control.c
 * @brief Implementation of the motor state machine and control loop.
 */

#include "motor_control.h"
#include "motor_config.h"
#include "motor_pwm.h"
#include "motor_can.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "MOTOR_CTRL";

// State Machine Definitions
typedef enum {
    STARTUP_STATE,
    OPERATION_STATE,
    CAN_OVER_UART_STATE
} motor_state_t;

static void ts_motorControlLoop(void *pvParameters) {
    motor_can_init();
    motor_pwm_init();

    motor_state_t state = STARTUP_STATE;
    uint8_t target_throttle = 0;
    bool waiting_for_zero = false;
    uint32_t zero_throttle_start_time = 0;

    while (1) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // 1. Fetch Data
        bool can_data_fault = false;
        target_throttle = motor_can_get_throttle(&can_data_fault);
        bool temp_is_high = motor_can_is_temp_high();
        bool comms_timeout = motor_can_is_timeout();

        // 2. Global Fault Evaluation
        bool system_fault_active = can_data_fault || temp_is_high || comms_timeout;

        if (system_fault_active && state == OPERATION_STATE) {
            ESP_LOGE(TAG, "System fault detected! Forcing STARTUP_STATE.");
            state = STARTUP_STATE;
        }

        // 3. State Machine Execution
        switch (state) {

            case STARTUP_STATE: // Error / Initialization State
                motor_pwm_stop(); // Force hardware to 0%

                if (!system_fault_active) {
                    if (target_throttle == 0) {
                        if (!waiting_for_zero) {
                            zero_throttle_start_time = now;
                            waiting_for_zero = true;
                        } else if (now - zero_throttle_start_time >= SAFE_STARTUP_WAIT_MS) {
                            state = OPERATION_STATE;
                            waiting_for_zero = false;
                            ESP_LOGI(TAG, "Neutral Interlock cleared. System ARMED.");
                        }
                    } else {
                        waiting_for_zero = false; // Throttle applied during wait time
                    }
                } else {
                    waiting_for_zero = false; // Fault is present, reset interlock timer
                }
                break;

            case OPERATION_STATE: // Normal Operation
                motor_pwm_set_throttle(target_throttle);
                break;

            case CAN_OVER_UART_STATE: // Fallback State
                motor_pwm_stop(); 
                // TODO: Implement actual CAN over UART fallback
                break;
        }

        // Loop delay (100Hz execution rate)
        vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS)); 
    }
}

void motor_control_start_task(void) {
    ESP_LOGI(TAG, "Starting Motor Control Task...");

    BaseType_t xReturned = xTaskCreate(
        ts_motorControlLoop,
        "motor_ctrl_task",
        4096,
        NULL,
        5,                      // Priority
        NULL
    );

    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create motor control task!");
    }
}
