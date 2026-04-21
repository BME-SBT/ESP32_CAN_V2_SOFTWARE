/**
 * @file motor_can.c
 * @brief High-level CAN application layer for the motor controller node.
 *
 * This module acts as the bridge between the low-level CAN hardware wrapper
 * and the motor control application logic. It initializes the CAN bus using
 * the universal wrapper, spawns a dedicated FreeRTOS background task to
 * continuously listen for incoming network traffic, and safely caches the
 * latest telemetry (throttle commands and motor temperatures) using mutexes.
 * It provides thread-safe getter functions for the main motor control loop
 * to access the freshest data without dropping hardware frames.
 */

#include "motor_can.h"
#include "motor_config.h"
#include "can_wrapper.h"
#include "can_protocol.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char* TAG = "MOTOR_CAN";

// Shared state variables accessed by both RX task and application logic
static volatile uint32_t last_valid_msg_time = 0;
static uint8_t current_throttle = 0;
static uint8_t current_temp = 0;
static bool throttle_length_fault = false;

// Mutex to protect shared state variables from race conditions
static SemaphoreHandle_t data_mutex;

// Background task to continuously process incoming CAN messages
static void motor_can_rx_task(void *arg) {
    (void)arg; // Cast to void to prevent unused parameter warning
    twai_message_t rx_msg;

    while (1) {
        // Block indefinitely until a message is received from the hardware
        if (can_receive(&rx_msg, portMAX_DELAY) == ESP_OK) {
            // Lock the shared data section before modifying it
            xSemaphoreTake(data_mutex, portMAX_DELAY);

            // Check if the received message matches the expected control ID
            if (rx_msg.identifier == Control_ID) {
                // Ensure the payload contains at least 1 byte of data
                if (rx_msg.data_length_code >= 1) {
                    current_throttle = rx_msg.data[0];
                    throttle_length_fault = false;
                    uint32_t t = xTaskGetTickCount();
                    last_valid_msg_time = t * portTICK_PERIOD_MS; // Convert RTOS ticks to milliseconds
                } else {
                    // Flag a fault if the message is empty
                    throttle_length_fault = true;
                }
            } 
            // Check if the received message matches the motor temperature ID
            else if (rx_msg.identifier == MotorTemps_ID) {
                if (rx_msg.data_length_code >= 1) {
                    current_temp = rx_msg.data[0];
                }
            }

            // Unlock the shared data section after modifications are complete
            xSemaphoreGive(data_mutex);
        }
    }
}

void motor_can_init(void) {
    // Initialize the mutex required for thread-safe variable access
    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create data mutex!");
        return;
    }

    // Configure the CAN driver settings
    can_config_t cfg = {
        .tx_pin = PIN_CAN_TX,
        .rx_pin = PIN_CAN_RX,
        .baud_rate = CAN_BAUD_500K,
        .mode = CAN_MODE_NORMAL
    };

    // Initialize the underlying CAN wrapper driver
    ESP_ERROR_CHECK(can_init(&cfg));

    // Pre-load the watchdog timer to give the system time to start up
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    last_valid_msg_time = now + WD_TIMEOUT_MS;

    // Spawn the background FreeRTOS task to handle reception
    xTaskCreate(
        motor_can_rx_task,
        "motor_can_rx",
        4096,
        NULL,
        10,
        NULL
    );

    ESP_LOGI(TAG, "Motor CAN Initialized. RX Task running.");
}

uint8_t motor_can_get_throttle(bool *out_fault) {
    // Lock shared data to read the current throttle state safely
    xSemaphoreTake(data_mutex, portMAX_DELAY);

    if (throttle_length_fault) {
        ESP_LOGE(TAG, "Throttle data length is less than expected!");
        *out_fault = true;
        xSemaphoreGive(data_mutex);
        return 0;
    }

    // Copy the value to a local variable to minimize lock time
    uint8_t raw_val = current_throttle;

    // Release the lock as soon as the data is safely copied
    xSemaphoreGive(data_mutex);

    // Validate that the requested throttle value is within safe limits
    if ((int16_t)raw_val < MIN_THROTTLE || raw_val > MAX_THROTTLE) {
        ESP_LOGE(TAG, "Throttle value out of bounds: %u", raw_val);
        *out_fault = true;
        return 0;
    }

    *out_fault = false;
    return raw_val;
}

bool motor_can_is_temp_high(void) {
    // Lock shared data to safely read the temperature
    xSemaphoreTake(data_mutex, portMAX_DELAY);
    uint8_t motor_temp = current_temp;
    xSemaphoreGive(data_mutex);

    // Check if the current temperature exceeds the defined maximum threshold
    if (motor_temp > MAX_MOTOR_TEMP) {
        ESP_LOGE(TAG, "Motor OVERTEMP: %u C", motor_temp);
        return true;
    }

    return false;
}

bool motor_can_is_timeout(void) {
    // Retrieve the current system time in milliseconds
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Return true if elapsed time since last valid message exceeds the limit
    return (now - last_valid_msg_time) > WD_TIMEOUT_MS;
}
