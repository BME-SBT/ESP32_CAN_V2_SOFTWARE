#include "can_manager.h"
#include "can_wrapper.h"
#include "esp_log.h"
#include "solar.h"

static const char *TAG = "CANMAN";

esp_err_t can_manager_init(void) {
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

  ESP_LOGI(TAG, "CAN started");
  return ESP_OK;
}

esp_err_t can_manager_transmit(const twai_message_t *message,
                               TickType_t ticks_to_wait) {
  if (!message) {
    return ESP_ERR_INVALID_ARG;
  }
  return can_send(message, pdTICKS_TO_MS(ticks_to_wait));
}

void can_manager_handle_recovery(void) {
  if (can_is_bus_off()) {
    ESP_LOGW(TAG, "Bus-off detected, initiating recovery");
    (void)can_recover_bus();
  }
}