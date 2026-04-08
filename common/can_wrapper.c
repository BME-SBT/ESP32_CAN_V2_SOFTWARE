// Implementation of the CAN wrapper

#include "can_wrapper.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "CAN_WRAPPER";
static bool is_initialized = false; // Prevents starting it twice by accident

esp_err_t can_init(const can_config_t *config) {
    if (is_initialized) {
        ESP_LOGW(TAG, "CAN is already running!");
        return ESP_ERR_INVALID_STATE;
    }

    if (!config) return ESP_ERR_INVALID_ARG;

    // Setup Pins and Mode
    twai_mode_t esp_mode = TWAI_MODE_NORMAL;
    if (config->mode == CAN_MODE_LISTEN_ONLY) esp_mode = TWAI_MODE_LISTEN_ONLY;
    if (config->mode == CAN_MODE_LOOPBACK)    esp_mode = TWAI_MODE_NO_ACK;

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)config->tx_pin, 
        (gpio_num_t)config->rx_pin, 
        esp_mode
    );

    // Setup Speed (Translating enum to ESP macros)
    twai_timing_config_t t_config;
    switch (config->baud_rate) {
        case CAN_BAUD_125K: t_config = TWAI_TIMING_CONFIG_125KBITS(); break;
        case CAN_BAUD_250K: t_config = TWAI_TIMING_CONFIG_250KBITS(); break;
        case CAN_BAUD_500K: t_config = TWAI_TIMING_CONFIG_500KBITS(); break;
        case CAN_BAUD_1M:   t_config = TWAI_TIMING_CONFIG_1MBITS();   break;
        default:            t_config = TWAI_TIMING_CONFIG_500KBITS(); break;
    }

    // Setup Filters (Accept everything for now)
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install and Start the driver
    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Install failed: %s", esp_err_to_name(err));
        return err;
    }

    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Start failed: %s", esp_err_to_name(err));
        twai_driver_uninstall();
        return err;
    }

    is_initialized = true;
    ESP_LOGI(TAG, "CAN started on TX:%d RX:%d", config->tx_pin, config->rx_pin);
    return ESP_OK;
}

esp_err_t can_send(const twai_message_t *message, uint32_t timeout_ms) {
    if (!is_initialized) return ESP_ERR_INVALID_STATE;
    // pdMS_TO_TICKS converts our normal milliseconds into RTOS "ticks"
    return twai_transmit(message, pdMS_TO_TICKS(timeout_ms));
}

esp_err_t can_receive(twai_message_t *message, uint32_t timeout_ms) {
    if (!is_initialized) return ESP_ERR_INVALID_STATE;
    return twai_receive(message, pdMS_TO_TICKS(timeout_ms));
}

esp_err_t can_deinit(void) {
    if (!is_initialized) return ESP_ERR_INVALID_STATE;
    
    esp_err_t err = twai_stop();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop TWAI driver: %s", esp_err_to_name(err));
        return err;
    }

    err = twai_driver_uninstall();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to uninstall TWAI driver: %s", esp_err_to_name(err));
        return err;
    }

    is_initialized = false;
    ESP_LOGI(TAG, "CAN stopped");
    return ESP_OK;
}

bool can_is_bus_off(void) {
    if (!is_initialized) return false;
    
    twai_status_info_t status_info;
    if (twai_get_status_info(&status_info) == ESP_OK) {
        return (status_info.state == TWAI_STATE_BUS_OFF);
    }
    return false;
}

esp_err_t can_recover_bus(void) {
    if (!is_initialized) return ESP_ERR_INVALID_STATE;
    
    // Asks the hardware to try and reconnect to the CAN network
    esp_err_t err = twai_initiate_recovery();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Trying to recover CAN bus...");
    }
    return err;
}
