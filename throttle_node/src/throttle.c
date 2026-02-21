#include "throttle.h"
// #include "driver/adc.h"
// #include "driver/twai.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// from yesweCAN.xslx
#define ID_THROTTLE 517

#define ADC_MIN 500  // ~0.4V
#define ADC_MAX 3500 // ~2.8V

void init_hardware(void) {
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); // GPIO 34

  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_22, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config =
      TWAI_TIMING_CONFIG_500KBITS(); //? TWAI at 500kbps but must be clarified
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();
}

throttle_data_t read_and_validate_throttle(void) {
  throttle_data_t result;
  int raw = adc1_get_raw(ADC1_CHANNEL_6);

  if (raw < ADC_MIN || raw > ADC_MAX) {
    result.percentage = 0;
    result.status = 0;
  } else {
    result.percentage = (int8_t)((raw - ADC_MIN) * 100 / (ADC_MAX - ADC_MIN));
    result.status = 1;
  }
  return result;
}

void can_transmit_task(void *pvParameters) {
  uint8_t counter = 0;
  TickType_t last_wake = xTaskGetTickCount();

  while (1) {
    throttle_data_t data = read_and_validate_throttle();

    twai_message_t msg = {.identifier = THROTTLE_CAN_ID,
                          .data_length_code = 8,
                          .data = {(uint8_t)data.percentage, data.status, 0, 0,
                                   0, 0, 0, counter}};

    if (twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK) {
      counter++;
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
  }
}
