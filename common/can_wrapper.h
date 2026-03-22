// CAN wrapper for all nodes in the solarboat

#ifndef CAN_WRAPPER_H
#define CAN_WRAPPER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#include "driver/twai.h" 

#ifdef __cplusplus
extern "C" {
#endif

// Setting options so nodes don't have to use  ESP32 macros
typedef enum {
    CAN_BAUD_125K,
    CAN_BAUD_250K,
    CAN_BAUD_500K, // Both throttle and motorbox nodes use this
} can_baud_rate_t;

typedef enum {
    CAN_MODE_NORMAL,       // Regular send/receive
    CAN_MODE_LISTEN_ONLY,  // Only read, don't send (good for debugging)
    CAN_MODE_LOOPBACK      // Tests if the ESP32 can talk to itself
} can_mode_t;

// The main configuration structure
typedef struct {
    int tx_pin;                // TX pin number
    int rx_pin;                // RX pin number
    can_baud_rate_t baud_rate; // Speed setting
    can_mode_t mode;           // Operating mode
} can_config_t;


// The simple API functions that replace twai_driver_install, twai_start, etc.

// Starts the CAN hardware
esp_err_t can_init(const can_config_t *config);

// Sends a message. "timeout_ms" is how long to wait if the bus is busy.
esp_err_t can_send(const twai_message_t *message, uint32_t timeout_ms);

// Waits to receive a message.
esp_err_t can_receive(twai_message_t *message, uint32_t timeout_ms);

// Shuts down the CAN hardware safely.
esp_err_t can_deinit(void);

// Checks if the CAN bus crashed due to too many errors (electrical noise, etc.)
bool can_is_bus_off(void);

// Tries to restart the CAN bus if it crashed.
esp_err_t can_recover_bus(void);

#ifdef __cplusplus
}
#endif

#endif // CAN_WRAPPER_H
