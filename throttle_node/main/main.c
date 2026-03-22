/**
 * @file main.c
 * @brief Application entry point for the throttle node
 * @author Jaloliddin Ismailov
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "throttle.h"

void app_main(void) {
  ESP_LOGI("MAIN", "Booting SBT throttle node...");

  if (init_hardware() != ESP_OK) {
    ESP_LOGE("MAIN", "Hardware init failed! Boat staying at the dock.");
    return;
  }

  // launch the transmission task
  xTaskCreate(can_transmit_task, "throttle_tx_task", 4096, NULL, 5, NULL);
}