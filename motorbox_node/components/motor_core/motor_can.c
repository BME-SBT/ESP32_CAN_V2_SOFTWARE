/**
 * @file motor_can.c
 * @brief Implementation of CAN frame handling and data validation.
 */

#include "motor_can.h"
#include "motor_config.h"
#include "can_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "MOTOR_CAN";

// State variables for validation and watchdog
static volatile uint32_t last_valid_msg_time = 0;

// Frames initialized with IDs from solar.h
// @TODO Remove after solar.h overhaul
static twai_message_t controll_frame = { .identifier = Control_ID, .data_length_code = 8, .data = {0} };
static twai_message_t motor_temps_frame = { .identifier = MotorTemps_ID, .data_length_code = 8, .data = {0} };
static twai_message_t motor_data_frame = { .identifier = MotorData_ID, .data_length_code = 8, .data = {0} };

// TWAI (CAN) Driver Configurations using pins from solar.h
static twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
static twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

void motor_can_init(void) {
    // Update watchdog, add extra time for system start
    last_valid_msg_time = (xTaskGetTickCount() * portTICK_PERIOD_MS) + WD_TIMEOUT_MS;

    // Register frames before init, as requested by can_manager.h
    ESP_ERROR_CHECK(can_manager_register_frame(&controll_frame, true, true));
    ESP_ERROR_CHECK(can_manager_register_frame(&motor_data_frame, true, true));
    ESP_ERROR_CHECK(can_manager_register_frame(&motor_temps_frame, true, true));

    // Initialize CAN manager with 100ms TX period
    ESP_ERROR_CHECK(can_manager_init(&g_config, &t_config, &f_config, 100, TAG));
    ESP_LOGI(TAG, "CAN Manager Initialized on TX:%d RX:%d", PIN_CAN_TX, PIN_CAN_RX);
}

uint8_t motor_can_get_throttle(bool *out_fault) {
    can_manager_lock_frame(&controll_frame);

    if (controll_frame.data_length_code < 1) {
        ESP_LOGE(TAG, "Throttle data length is less than expected!");
        *out_fault = true;
        can_manager_unlock_frame(&controll_frame);
        return 0;
    }

    uint8_t raw_val = controll_frame.data[0];
    can_manager_unlock_frame(&controll_frame);

    if ((int16_t)raw_val < MIN_THROTTLE || raw_val > MAX_THROTTLE) {
        ESP_LOGE(TAG, "Throttle value out of bounds: %u", raw_val);
        *out_fault = true;
        return 0;
    }

    *out_fault = false;
    last_valid_msg_time = xTaskGetTickCount() * portTICK_PERIOD_MS; // Update watchdog
    return raw_val;
}

bool motor_can_is_temp_high(void) {
    can_manager_lock_frame(&motor_temps_frame);

    if (motor_temps_frame.data_length_code < 1) {
        can_manager_unlock_frame(&motor_temps_frame);
        return false; 
    }

    uint8_t motor_temp = motor_temps_frame.data[0];
    can_manager_unlock_frame(&motor_temps_frame);

    if (motor_temp > MAX_MOTOR_TEMP) {
        ESP_LOGE(TAG, "Motor OVERTEMP: %u C", motor_temp);
        return true;
    }
    return false;
}

bool motor_can_is_timeout(void) {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return (now - last_valid_msg_time) > WD_TIMEOUT_MS;
}
