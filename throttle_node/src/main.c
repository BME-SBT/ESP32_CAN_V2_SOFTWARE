#include "throttle.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
void app_main(void) {
    init_hardware();

    xTaskCreate(can_transmit_task, "throttle_tx", 4096, NULL, 5, NULL);
}