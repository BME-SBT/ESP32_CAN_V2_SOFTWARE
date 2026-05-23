#pragma once
#ifndef CAN_MANAGER_H_
#define CAN_MANAGER_H_

#include <stdint.h>

#include "driver/twai.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t can_manager_init(void);

esp_err_t can_manager_transmit(const twai_message_t *message,
                               uint32_t timeout_ms);

void can_manager_handle_recovery(void);

#ifdef __cplusplus
}
#endif

#endif // CAN_MANAGER_H_
