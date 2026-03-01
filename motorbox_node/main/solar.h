#pragma once
#ifndef MAIN_SOLAR_H_
#define MAIN_SOLAR_H_
#include "driver/gpio.h"

// start of pin deffinitions
#define PIN_GPIO_0 GPIO_NUM_1
#define PIN_GPIO_1 GPIO_NUM_0
#define PIN_GPIO_2 GPIO_NUM_7
#define PIN_GPIO_3 GPIO_NUM_6
#define PIN_GPIO_4 GPIO_NUM_5
#define PIN_GPIO_5 GPIO_NUM_4
#define PIN_GPIO_6 GPIO_NUM_16
#define PIN_GPIO_7 GPIO_NUM_9
#define PIN_GPIO_8 GPIO_NUM_18
#define PIN_GPIO_9 GPIO_NUM_19
#define PIN_GPIO_10 GPIO_NUM_20
#define PIN_GPIO_11 GPIO_NUM_21
#define PIN_GPIO_12 GPIO_NUM_22
#define PIN_GPIO_13 GPIO_NUM_23
#define PIN_GPIO_14 GPIO_NUM_15
#define PIN_GPIO_15 GPIO_NUM_17
#define PIN_CAN_RX GPIO_NUM_3
#define PIN_CAN_TX GPIO_NUM_2
#define PIN_CLK GPIO_NUM_8
#define PIN_LATCH GPIO_NUM_10
#define PIN_DATA GPIO_NUM_11
// end of pin deffinitions

#define MotorTemps_ID	0x200
#define MotorData_ID	0x201
#define BMSData_ID	    0x300
#define BMSExtra_ID	    0x301
#define GPS_ID	        0x400
#define Position_ID	    0x401
#define Control_ID	    0x100


// used instead of gpio_set_direction, this on actually works
void gpio_set_dir(int gpio, int mode) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};

    if (mode == 2) {
        io_conf.mode = GPIO_MODE_OUTPUT;
    }
    if (mode == 1) {
        io_conf.mode = GPIO_MODE_INPUT;
    }

    gpio_config(&io_conf);
}


#endif /* MAIN_SOLAR_H_ */