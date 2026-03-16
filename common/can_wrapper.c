#include "can_wrapper.h"
#include "esp_log.h"

static const char *TAG = "CAN_WRAPPER";

bool can_init(const can_config_t *config) {
    // Standard configuration extracted from Jalo's code
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(config->tx_io, config->rx_io, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); 
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install driver");
        return false;
    }
    
    if (twai_start() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start driver");
        return false;
    }
    
    return true;
}

bool can_send(uint32_t id, const uint8_t *data, uint8_t len) {
    twai_message_t message = {
        .identifier = id,
        .data_length_code = (len > 8) ? 8 : len,
        .ss = 1 // Self-reception or standard transmit logic
    };

    for (int i = 0; i < message.data_length_code; i++) {
        message.data[i] = data[i];
    }

    return (twai_transmit(&message, pdMS_TO_TICKS(10)) == ESP_OK);
}

bool can_receive(twai_message_t *msg) {
    // Non-blocking receive for use in task loops like Koxi's
    return (twai_receive(msg, 0) == ESP_OK);
}
