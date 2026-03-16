#ifndef CAN_WRAPPER_H
#define CAN_WRAPPER_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/twai.h"

// Define standard pins based on Koxi's solar.h
#define DEFAULT_CAN_TX PIN_CAN_TX 
#define DEFAULT_CAN_RX PIN_CAN_RX

typedef struct {
    int tx_io;
    int rx_io;
    uint32_t baud_rate; // e.g. 500 for 500kbits
} can_config_t;

// Standard API requested by leader
bool can_init(const can_config_t *config);
bool can_send(uint32_t id, const uint8_t *data, uint8_t len);
bool can_receive(twai_message_t *msg);

#endif
