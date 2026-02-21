#ifndef THROTTLE_H
#define THROTTLE_H

#include <stdint.h>

//! configs from yesweCAN.xslx
#define THROTTLE_CAN_ID 517
#define THROTTLE_ADC_PIN 34
#define ADC_MIN 500
#define ADC_MAX 3500

typedef struct throttle_data_t {
  int8_t percentage;
  uint8_t status;
} throttle_data_t;

void init_hardware(void);
void can_transmit_task(void *pvParameters);

#endif