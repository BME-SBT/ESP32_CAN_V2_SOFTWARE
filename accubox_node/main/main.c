/**
 * @file main.c
 * @brief Application entry point for the accubox node.
 */

#include "accubox.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void) {
    ESP_LOGI("MAIN", "Booting SBT accubox node...");

    if (accubox_init() != ESP_OK) {
        ESP_LOGE("MAIN", "Accubox init failed! CAN transmit disabled.");
        return;
    }

    xTaskCreate(accubox_transmit_task, "accubox_tx_task", 4096, NULL, 5, NULL);
}
