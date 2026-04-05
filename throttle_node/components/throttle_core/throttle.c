/**
 * @file throttle.c
 * @brief Implementation of throttle validation.
 * @author Jaloliddin Ismailov
 */

#include "throttle.h"

#include "can_wrapper.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "THROTTLE";

// persists the error state between loops so telemetry reflects last CAN failure
static uint8_t persistent_can_error_flag = 0;
static uint32_t can_error_counter __attribute__((unused)) =
    0; // this is not used yet

esp_err_t init_hardware(void) {
  // ADC setup for GPIO 34 (ADC1_CH6)
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);

  can_config_t can_config = {
      .tx_pin = PIN_CAN_TX,
      .rx_pin = PIN_CAN_RX,
      .baud_rate = CAN_BAUD_500K,
      .mode = CAN_MODE_NORMAL,
  };

  esp_err_t err = can_init(&can_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "CAN init failed: %s", esp_err_to_name(err));
    return err;
  }

  ESP_LOGI(TAG, "Boat ready to sail!");
  return ESP_OK;
}

throttle_data_t read_and_validate_throttle(void) {
  throttle_data_t result = {.percentage = 0, .status = 0};
  int raw = adc1_get_raw(ADC1_CHANNEL_6);

  // range check: if sensor is disconnected or shorted, flag it
  if (raw < ADC_MIN || raw > ADC_MAX) {
    result.status |= STATUS_ADC_OUT_OF_RANGE;
    ESP_LOGD(TAG, "ADC out of range: %d", raw);
  } else {
    // linear scaling 0-100%
    int32_t scaled = (raw - ADC_MIN) * 100 / (ADC_MAX - ADC_MIN);

    if (scaled > 100) {
      scaled = 100;
    }
    if (scaled < 0) {
      scaled = 0;
    }

    result.percentage = (int8_t)scaled;
    result.status |= STATUS_VALID;
  }
  return result;
}

void can_transmit_task(void *pvParameters) {
  uint8_t msg_counter = 0;
  (void)pvParameters;

  // initialise watchdog BEFORE adding current task
  esp_task_wdt_init(2, true);
  esp_task_wdt_add(NULL);

  ESP_LOGI(TAG, "CAN Transmit Task live at 50Hz");

  while (1) {
    esp_task_wdt_reset();

    throttle_data_t data = read_and_validate_throttle();
    data.status |= persistent_can_error_flag;

    twai_message_t msg = {.identifier = THROTTLE_CAN_ID,
                          .data_length_code = 8,
                          .data = {(uint8_t)data.percentage, data.status, 0, 0,
                                   0, 0, 0, msg_counter}};

    if (can_send(&msg, 10) == ESP_OK) {
      persistent_can_error_flag &= (uint8_t)(~STATUS_CAN_ERROR);
      msg_counter++;
    } else {
      persistent_can_error_flag |= STATUS_CAN_ERROR;
      can_error_counter++;
      if (can_is_bus_off()) {
        ESP_LOGW(TAG, "Bus-off detected, initiating recovery");
        (void)can_recover_bus();
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20)); // keep it steady at 50Hz
  }
}
