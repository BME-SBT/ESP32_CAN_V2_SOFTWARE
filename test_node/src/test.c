/**
 * @file test.c
 * @brief Test node implementation - simulates throttle node with fault injection.
 *
 * Structure:
 *   - test_init_hardware()   -> (CAN only, no ADC)
 *   - can_transmit_task      -> same name, same frame layout, adds fault injection
 *   - test_rx_task           -> logs incoming frames from the motorbox
 *   - mode_switch_task       -> cycles through fault modes every 5 seconds
 */
 
#include "test.h"
#include "can_manager.h"
#include "solar.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
 
// Status byte bitmasks
#define STATUS_VALID            0x01
#define STATUS_ADC_OUT_OF_RANGE 0x02
#define STATUS_CAN_ERROR        0x04
 
// CAN ID for throttle messages - from solar.h
#define THROTTLE_CAN_ID Control_ID

 
#define TX_PERIOD_MS     20    // Normal send period: 20ms = 50Hz
#define MODE_DURATION_MS 5000  // How long each test mode runs before switching
 
static const char *TAG = "TEST_NODE";
 
static volatile test_mode_t current_mode = TEST_MODE_NORMAL;
 
// Mirrors the persistent error tracking in throttle.c
static uint8_t persistent_can_error_flag = 0;
static uint32_t can_error_counter = 0;
 
// For logging
static const char *mode_names[] = {
    "NORMAL", "INVALID_THROTTLE", "BAD_STATUS", "WRONG_COUNTER",
    "TIMEOUT", "SLOW", "JITTER", "RANDOM"
};
 

// Mirrors init_hardware() in throttle.c, without ADC
static esp_err_t test_init_hardware(void) {
    esp_err_t err = can_manager_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CAN init failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "Test node ready");
    return ESP_OK;
}
 

// Instead of reading a real ADC, it generates values based on the current test mode. 
static void build_frame_data(uint8_t *throttle_out, uint8_t *status_out,
                              uint8_t counter, uint8_t *data_out, bool *skip_out)
{
    // Default: valid throttle at 50%, status valid
    uint8_t throttle    = 50;
    uint8_t status      = STATUS_VALID | persistent_can_error_flag;
    uint8_t seq_counter = counter;
    *skip_out = false;
 
    switch (current_mode) {
        case TEST_MODE_INVALID_THROTTLE:
            throttle = 200; // Outside 0-100 range sent by real throttle node
            status = STATUS_ADC_OUT_OF_RANGE | persistent_can_error_flag;
            // status = STATUS_VALID | persistent_can_error_flag; // This might be better for testing
            break;
 
        case TEST_MODE_BAD_STATUS:
            status = 0; // No valid bit set
            break;
 
        case TEST_MODE_WRONG_COUNTER:
            seq_counter = counter + 10; // Jump the counter
            break;
 
        case TEST_MODE_TIMEOUT:
            *skip_out = true; // Doesn't send anything - watchdog will trigger
            return;
 
        case TEST_MODE_SLOW:
            vTaskDelay(pdMS_TO_TICKS(200 - TX_PERIOD_MS)); // Extra delay on top of base period
            break;
 
        case TEST_MODE_JITTER:
            vTaskDelay(pdMS_TO_TICKS(rand() % 150)); // Random extra delay
            break;
 
        case TEST_MODE_RANDOM:
            for (int i = 0; i < 8; i++) data_out[i] = (uint8_t)(rand() % 256); // Fully random 8-byte payload
            return; // data_out already filled, skip structured fill below
 
        default:
            break;
    }
 
    // Structured frame
    data_out[0] = throttle;
    data_out[1] = status;
    data_out[2] = 0;
    data_out[3] = 0;
    data_out[4] = 0;
    data_out[5] = 0;
    data_out[6] = 0;
    data_out[7] = seq_counter;
 
    *throttle_out = throttle;
    *status_out   = status;
}
 

// TX task
static void can_transmit_task(void *pvParameters) {
    uint8_t msg_counter = 0;
    (void)pvParameters;
 
    esp_task_wdt_init(30, true);
    esp_task_wdt_add(NULL);
 
    ESP_LOGI(TAG, "CAN Transmit Task live at 50Hz");
 
    while (1) {
        esp_task_wdt_reset();
 
        twai_message_t msg = {
            .identifier       = THROTTLE_CAN_ID,
            .data_length_code = 8,
            .data             = {0}
        };
 
        uint8_t throttle = 0, status = 0;
        bool skip = false;
 
        build_frame_data(&throttle, &status, msg_counter, msg.data, &skip);
 
        if (skip) {
            ESP_LOGW(TAG, "[TIMEOUT] Not sending - waiting for motorbox watchdog...");
            vTaskDelay(pdMS_TO_TICKS(TX_PERIOD_MS));
            continue;
        }
 
        // Transmit - same call as throttle.c
        if (can_manager_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK) {
            persistent_can_error_flag &= (uint8_t)(~STATUS_CAN_ERROR);
            ESP_LOGI(TAG, "TX [%s] throttle=%d status=0x%02X counter=%d",
                     mode_names[current_mode], throttle, status, msg_counter);
            msg_counter++;
        } else {
            persistent_can_error_flag |= STATUS_CAN_ERROR;
            can_error_counter++;
            ESP_LOGW(TAG, "TX failed (error count: %lu)", can_error_counter);
            can_manager_handle_recovery();
        }
 
        vTaskDelay(pdMS_TO_TICKS(TX_PERIOD_MS)); // 50Hz base rate
    }
}
 

// RX task - logs any incoming frames (e.g. from motorbox) - for observability during testing 
static void test_rx_task(void *pvParameters) {
    (void)pvParameters;
    twai_message_t rx_msg;
 
    while (1) {
        // Blocks up to 1 second waiting for a message
        if (can_manager_receive(&rx_msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
            ESP_LOGI(TAG, "RX id=0x%03X dlc=%d data=[%02X %02X %02X %02X ...]",
                     rx_msg.identifier, rx_msg.data_length_code,
                     rx_msg.data[0], rx_msg.data[1],
                     rx_msg.data[2], rx_msg.data[3]);
        }
    }
}
 

// Mode switch task - cycles through test modes automatically
static void mode_switch_task(void *pvParameters) {
    (void)pvParameters;
 
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(MODE_DURATION_MS));
        current_mode = (test_mode_t)((int)(current_mode + 1) % TEST_MODE_COUNT);
        ESP_LOGI(TAG, "==============================");
        ESP_LOGI(TAG, "  NEW MODE [%d]: %s", current_mode, mode_names[current_mode]);
        ESP_LOGI(TAG, "==============================");
    }
}
 

// Public init
void test_init(void) {
    ESP_LOGI(TAG, "Test node starting...");
    ESP_ERROR_CHECK(test_init_hardware());
 
    xTaskCreate(can_transmit_task, "test_tx",     4096, NULL, 5, NULL);
    xTaskCreate(test_rx_task,      "test_rx",     2048, NULL, 3, NULL);
    xTaskCreate(mode_switch_task,  "mode_switch", 2048, NULL, 1, NULL);
}