/**
 * @file accubox.c
 * @brief Accubox CAN telemetry for BMS data.
 */

#include "accubox.h"

#include "can_manager.h"
#include "can_protocol.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "ACCUBOX";

#define ACCUBOX_TX_PERIOD_MS 100
#define ACCUBOX_TX_TIMEOUT_TICKS pdMS_TO_TICKS(10)

typedef struct __attribute__((packed)) {
    bms_data_msg_t data;
    int16_t ebox_air_temperature;
} bms_data_payload_t;

typedef struct __attribute__((packed)) {
    bms_extra_msg_t data;
    int16_t mppt_temperature;
} bms_extra_payload_t;

typedef struct {
    int16_t voltage;
    int16_t current;
    uint16_t soc;
    int16_t temperature;
    int16_t mppt_current;
    int16_t mppt_temperature;
    int16_t fan_rpm;
    int16_t ebox_air_temperature;
} accubox_measurements_t;

static accubox_measurements_t accubox_read_measurements(void) {
    accubox_measurements_t measurements = {0};
    return measurements;
}

static void accubox_build_frames(twai_message_t *bms_data_msg,
                                 twai_message_t *bms_extra_msg) {
    accubox_measurements_t measurements = accubox_read_measurements();

    bms_data_payload_t data_payload = {
        .data = {
            .voltage = measurements.voltage,
            .current = measurements.current,
            .soc = measurements.soc,
        },
        .ebox_air_temperature = measurements.ebox_air_temperature,
    };

    bms_extra_payload_t extra_payload = {
        .data = {
            .temperature = measurements.temperature,
            .mppt_current = measurements.mppt_current,
            .fan_rpm = measurements.fan_rpm,
        },
        .mppt_temperature = measurements.mppt_temperature,
    };

    *bms_data_msg = (twai_message_t){
        .identifier = BMSData_ID,
        .data_length_code = sizeof(data_payload),
        .data = {0},
    };
    memcpy(bms_data_msg->data, &data_payload, sizeof(data_payload));

    *bms_extra_msg = (twai_message_t){
        .identifier = BMSExtra_ID,
        .data_length_code = sizeof(extra_payload),
        .data = {0},
    };
    memcpy(bms_extra_msg->data, &extra_payload, sizeof(extra_payload));
}

esp_err_t accubox_init(void) {
    esp_err_t err = can_manager_init();
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "Accubox node initialized");
    return ESP_OK;
}

void accubox_transmit_task(void *pvParameters) {
    (void)pvParameters;

    ESP_LOGI(TAG, "Accubox CAN transmit task started");

    while (1) {
        twai_message_t bms_data_msg;
        twai_message_t bms_extra_msg;

        accubox_build_frames(&bms_data_msg, &bms_extra_msg);

        esp_err_t data_err = can_manager_transmit(&bms_data_msg,
                                                  ACCUBOX_TX_TIMEOUT_TICKS);
        if (data_err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to transmit BMS data: %s",
                     esp_err_to_name(data_err));
            can_manager_handle_recovery();
        }

        esp_err_t extra_err = can_manager_transmit(&bms_extra_msg,
                                                   ACCUBOX_TX_TIMEOUT_TICKS);
        if (extra_err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to transmit BMS extra data: %s",
                     esp_err_to_name(extra_err));
            can_manager_handle_recovery();
        }

        vTaskDelay(pdMS_TO_TICKS(ACCUBOX_TX_PERIOD_MS));
    }
}
