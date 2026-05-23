/**
 * @file accubox.h
 * @brief Accubox telemetry task for BMS data.
 */

#ifndef ACCUBOX_H
#define ACCUBOX_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t accubox_init(void);
void accubox_transmit_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // ACCUBOX_H
