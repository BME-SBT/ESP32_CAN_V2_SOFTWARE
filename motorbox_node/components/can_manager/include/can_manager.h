#pragma once
#ifndef CAN_MANAGER_H_
#define CAN_MANAGER_H_

#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

// Registry auto-grows on first registration; register frames BEFORE can_manager_init().

// Initialize the CAN manager.
// tx_period_ms: fixed transmission period (ms) applied to all TX-enabled frames.
// Note: All config pointers (g, t, f) must be non-NULL and valid TWAI configs.
esp_err_t can_manager_init(const twai_general_config_t *g,
                           const twai_timing_config_t *t,
                           const twai_filter_config_t *f,
                           uint8_t tx_period_ms, const char *tag);

// Register a frame with the manager. The manager creates a per-frame mutex internally.
// tx_enable: whether this frame should be transmitted periodically.
// rx_enable: whether this frame should be updated by RX when a matching ID is received.
esp_err_t can_manager_register_frame(twai_message_t *frame, bool tx_enable, bool rx_enable);

// Update TX/RX settings for a registered frame.
esp_err_t can_manager_set_tx_enable(twai_message_t *frame, bool tx_enable);
esp_err_t can_manager_set_rx_enable(twai_message_t *frame, bool rx_enable);

// Lock/unlock a registered frame for safe mutation by application code.
esp_err_t can_manager_lock_frame(twai_message_t *frame);
esp_err_t can_manager_unlock_frame(twai_message_t *frame);

// Get debounced error state.
bool can_manager_error_get(void);

// Optional: deinitialize and stop tasks/driver (not mandatory for basic usage).
esp_err_t can_manager_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // CAN_MANAGER_H_
