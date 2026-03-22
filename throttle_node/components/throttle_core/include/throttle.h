/**
 * @file throttle.h
 * @brief Throttle sensing and CAN telemetry
 * @author Jaloliddin Ismailov
 */

#ifndef THROTTLE_H
#define THROTTLE_H

#include "../../../../common/can_protocol.h"
#include "esp_err.h"
#include "solar.h"
#include <stdint.h>

// adjust these based on your specific hall effect sensor
#define ADC_MIN 500
#define ADC_MAX 3500

// status bitmasks for telemetry
#define STATUS_VALID 0x01
#define STATUS_ADC_OUT_OF_RANGE 0x02
#define STATUS_CAN_ERROR 0x04

// use the ID from the protocol table
#ifndef THROTTLE_CAN_ID
#define THROTTLE_CAN_ID Control_ID
#endif // THROTTLE_CAN_ID

typedef struct {
  int8_t percentage;
  uint8_t status;
} throttle_data_t;

/**
 * @brief Initialise ADC and TWAI hardware
 */
esp_err_t init_hardware(void);

/**
 * @brief Main transmission task (50Hz)
 */
void can_transmit_task(void *pvParameters);

#endif // THROTTLE_H
