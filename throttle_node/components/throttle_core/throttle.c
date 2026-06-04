/**
 * @file throttle.c
 * @brief Implementation of throttle validation.
 */

#include "throttle.h"

#include "can_wrapper.h"
#include "esp_adc/adc_oneshot.h" // ez lett
// #include "driver/adc.h" // ehelyett
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "THROTTLE";

// persists CAN error state between loops
static uint8_t persistent_can_error_flag = 0;
static uint32_t can_error_counter __attribute__((unused)) = 0;

static adc_oneshot_unit_handle_t adc_handle; // itt

esp_err_t init_hardware(void)
{
    /*adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_12);*/

    adc_oneshot_unit_init_cfg_t unit_cfg = { // innen
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_6, &chan_cfg)); // idaig
    
    can_config_t can_config = {
        .tx_pin = PIN_CAN_TX,
        .rx_pin = PIN_CAN_RX,
        .baud_rate = CAN_BAUD_500K,
        //.mode = CAN_MODE_NORMAL,
        .mode = CAN_MODE_LOOPBACK,
    };

    esp_err_t err = can_init(&can_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CAN init failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Throttle node initialized");
    return ESP_OK;
}

throttle_data_t read_and_validate_throttle(void)
{
    /*throttle_data_t result = {
        .percentage = 0,
        .status = 0
    };

    // int raw = adc1_get_raw(ADC1_CHANNEL_6); // ezt csereltuk

    int raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_6, &raw)); // erre a ket sorra

    if (raw < ADC_MIN || raw > ADC_MAX) {
        result.status |= STATUS_ADC_OUT_OF_RANGE;
        ESP_LOGD(TAG, "ADC out of range: %d", raw);
        return result;
    }

    int32_t scaled =
        (raw - ADC_MIN) * 100 / (ADC_MAX - ADC_MIN);

    if (scaled < 0) scaled = 0;
    if (scaled > 100) scaled = 100;

    result.percentage = (int8_t)scaled;
    result.status |= STATUS_VALID;

    return result;*/

    static int fake_value = ADC_MIN;
    static int direction = 1;

    throttle_data_t result = {
        .percentage = 0,
        .status = STATUS_VALID
    };

    // Szimulált ADC érték
    fake_value += direction * 50;

    if (fake_value >= ADC_MAX) {
        fake_value = ADC_MAX;
        direction = -1;
    }

    if (fake_value <= ADC_MIN) {
        fake_value = ADC_MIN;
        direction = 1;
    }

    int32_t scaled =
        (fake_value - ADC_MIN) * 100 / (ADC_MAX - ADC_MIN);

    if (scaled > 100) scaled = 100;
    if (scaled < 0) scaled = 0;

    result.percentage = (int8_t)scaled;

    ESP_LOGI(TAG, "SIM throttle: %d%%", result.percentage);

    return result;
}

void can_transmit_task(void *pvParameters)
{
    (void)pvParameters;
    uint8_t msg_counter = 0;

    /* ---------------- WATCHDOG (ESP-IDF v5.x style) ---------------- */

    /*esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 2000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true
    };

    esp_err_t wdt_err = esp_task_wdt_init(&twdt_config);
    if (wdt_err != ESP_OK && wdt_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "WDT init failed: %s", esp_err_to_name(wdt_err));
    }

    esp_task_wdt_add(NULL);*/

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    ESP_LOGI(TAG, "CAN Transmit Task started (50Hz)");

    while (1) {

        esp_task_wdt_reset();

        throttle_data_t data = read_and_validate_throttle();

        data.status |= persistent_can_error_flag;

        twai_message_t msg = {
            .identifier = THROTTLE_CAN_ID,
            .data_length_code = 8,
            .data = {
                (uint8_t)data.percentage,
                data.status,
                0, 0, 0, 0, 0,
                msg_counter
            }
        };

        if (can_send(&msg, 10) == ESP_OK) {
            persistent_can_error_flag &= (uint8_t)(~STATUS_CAN_ERROR);
            msg_counter++;
            ESP_LOGI(TAG, "SENT");
        } else {
            persistent_can_error_flag |= STATUS_CAN_ERROR;
            can_error_counter++;

            if (can_is_bus_off()) {
                ESP_LOGW(TAG, "CAN bus-off detected → recovery");
                (void)can_recover_bus();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz
    }
}