/**
 * @file motor_can.c
 * @brief High-level CAN application layer for the motor controller node.
 *
 * This module acts as the bridge between the low-level CAN hardware wrapper
 * and the motor control application logic. It initializes the CAN bus using
 * the universal wrapper, spawns a dedicated FreeRTOS background task to
 * continuously listen for incoming network traffic, and safely caches the
 * latest telemetry (throttle commands and motor temperatures) using mutexes.
 * It provides thread-safe getter functions for the main motor control loop
 * to access the freshest data without dropping hardware frames.
 */

#include "motor_can.h"
#include "motor_config.h"
#include "can_wrapper.h"
#include "can_protocol.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char* TAG = "MOTOR_CAN";

static volatile uint32_t last_valid_msg_time = 0;
static uint8_t current_throttle = 0;
static uint8_t current_enable = 0;
static int16_t current_temp = 0;
static bool throttle_length_fault = false;

static SemaphoreHandle_t data_mutex;

static void motor_can_rx_task(void *arg) {
    (void)arg;
    twai_message_t rx_msg;

    while (1) {
        if (can_receive(&rx_msg, portMAX_DELAY) == ESP_OK) {
            xSemaphoreTake(data_mutex, portMAX_DELAY);

            if (rx_msg.identifier == Control_ID) {
                if (rx_msg.data_length_code >= sizeof(control_msg_t)) {
                    control_msg_t *ctrl = (control_msg_t*)rx_msg.data; // Type-pun payload array to map bytes to struct
                    current_throttle = ctrl->throttle;
                    current_enable = ctrl->enable;
                    throttle_length_fault = false;
                    uint32_t t = xTaskGetTickCount();
                    last_valid_msg_time = t * portTICK_PERIOD_MS;
                } else {
                    throttle_length_fault = true;
                }
            } 
            else if (rx_msg.identifier == MotorTemps_ID) {
                if (rx_msg.data_length_code >= sizeof(motor_temps_msg_t)) {
                    motor_temps_msg_t *temps = (motor_temps_msg_t*)rx_msg.data; // Type-pun payload array to map bytes to struct
                    current_temp = temps->motor_temp;
                }
            }

            xSemaphoreGive(data_mutex);
        }
    }
}

void motor_can_init(void) {
    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create data mutex!");
        return;
    }

    can_config_t cfg = {
        .tx_pin = PIN_CAN_TX,
        .rx_pin = PIN_CAN_RX,
        .baud_rate = CAN_BAUD_500K,
        .mode = CAN_MODE_NORMAL
    };

    ESP_ERROR_CHECK(can_init(&cfg));

    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    last_valid_msg_time = now + WD_TIMEOUT_MS;

    xTaskCreate(motor_can_rx_task, "motor_can_rx", 4096, NULL, 10, NULL);
    ESP_LOGI(TAG, "Motor CAN Initialized. RX Task running.");
}

uint8_t motor_can_get_throttle(bool *out_fault) {
    xSemaphoreTake(data_mutex, portMAX_DELAY);

    if (throttle_length_fault) {
        *out_fault = true;
        xSemaphoreGive(data_mutex);
        return 0;
    }

    uint8_t raw_val = current_throttle;
    xSemaphoreGive(data_mutex);

    if ((int16_t)raw_val < MIN_THROTTLE || raw_val > MAX_THROTTLE) {
        *out_fault = true;
        return 0;
    }

    *out_fault = false;
    return raw_val;
}

bool motor_can_get_enable(void) {
    xSemaphoreTake(data_mutex, portMAX_DELAY);
    bool enabled = (current_enable > 0);
    xSemaphoreGive(data_mutex);
    return enabled;
}

bool motor_can_is_temp_high(void) {
    xSemaphoreTake(data_mutex, portMAX_DELAY);
    int16_t motor_temp = current_temp;
    xSemaphoreGive(data_mutex);

    return (motor_temp > MAX_MOTOR_TEMP);
}

bool motor_can_is_timeout(void) {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return (now - last_valid_msg_time) > WD_TIMEOUT_MS;
}

void motor_can_send_data(int16_t rpm, int16_t current, int16_t torque) {
    twai_message_t msg = {
        .identifier = MotorData_ID,
        .data_length_code = sizeof(motor_data_msg_t),
        .extd = 0,
        .rtr = 0
    };
    motor_data_msg_t *data = (motor_data_msg_t*)msg.data; // Type-pun struct onto array for automatic serialization
    data->rpm = rpm;
    data->current = current;
    data->torque = torque;

    can_send(&msg, 10);
}

void motor_can_send_temps(int16_t m_temp, int16_t c_temp) {
    twai_message_t msg = {
        .identifier = MotorTemps_ID,
        .data_length_code = sizeof(motor_temps_msg_t),
        .extd = 0,
        .rtr = 0
    };
    motor_temps_msg_t *data = (motor_temps_msg_t*)msg.data; // Type-pun struct onto array for automatic serialization
    data->motor_temp = m_temp;
    data->controller_temp = c_temp;

    can_send(&msg, 10);
}
