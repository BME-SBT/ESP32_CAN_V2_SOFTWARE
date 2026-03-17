/**
 * @file main.c
 * @brief Application entry point for the SolarBoat MotorBox.
 */

#include "esp_log.h"
#include "motor_control.h"

void app_main(void) {
    ESP_LOGI("MAIN", "SolarBoat MotorBox Booting...");

    // Start the FreeRTOS motor control task
    motor_control_start_task();
}
