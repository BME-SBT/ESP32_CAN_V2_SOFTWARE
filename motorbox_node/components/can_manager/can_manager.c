#include "can_manager.h"

// CONCERN: The can_manager errors will always have the same tags on all boards in our system.
static const char *TAG;

typedef struct {
    twai_message_t *frame;           // application-owned frame storage
    SemaphoreHandle_t m_frame;       // per-frame mutex (owned by manager)
    bool tx_enable;                  // whether periodic TX is enabled
    bool rx_enable;                  // whether RX is enabled
} canman_entry_t;

static canman_entry_t *frames_registry = NULL;           // registry of managed frames (heap)
static size_t frames_capacity = 0;    // total capacity
static size_t frames_count = 0;       // number of registered frames
static uint16_t g_tx_period_ms = 100; // default global; overridden in init

// Error handling state (debounced), protected by mutex
static volatile bool can_error = false;
static volatile uint8_t can_error_counter = 0;
static SemaphoreHandle_t m_can_error = NULL;

// TWAI driver running flag and task handles
static volatile bool can_running = false;

// Helper: error manipulation (same policy as existing main.c)
static inline void can_error_set(void) {
    if (m_can_error) xSemaphoreTake(m_can_error, portMAX_DELAY);
    if (can_error_counter < 10) can_error_counter++;
    can_error = true;
    if (m_can_error) xSemaphoreGive(m_can_error);
}

static inline void can_error_clear_one(void) {
    if (m_can_error) xSemaphoreTake(m_can_error, portMAX_DELAY);
    if (can_error_counter > 0) can_error_counter--;
    if (can_error_counter == 0) can_error = false;
    if (m_can_error) xSemaphoreGive(m_can_error);
}

static canman_entry_t *find_entry_by_frame(twai_message_t *frame) {
    for (size_t i = 0; i < frames_count; ++i) {
        if (frames_registry[i].frame == frame) return &frames_registry[i];
    }
    return NULL;
}

// Possible issue with this task is, if one frame's tx takes too long, it could 
// delay others. So not all frames might be sent exactly at the intended period.
// Prithul needs to fix this if any issue arises.
static void canman_tx_task(void *arg) {
    (void)arg;
    while (can_running) {
        for (size_t i = 0; i < frames_count; ++i) {
            canman_entry_t *e = &frames_registry[i];
            if (!e->tx_enable) continue;
            // Copy to local variable under mutex to minimize lock time
            twai_message_t local;
            xSemaphoreTake(e->m_frame, portMAX_DELAY);
            local = *(e->frame);
            xSemaphoreGive(e->m_frame);

            esp_err_t r = twai_transmit(&local, pdMS_TO_TICKS(10));
            if (r != ESP_OK) {
                ESP_LOGW(TAG, "Failed to transmit message id=0x%03X: %s", local.identifier, esp_err_to_name(r));
                can_error_set();
            } else {
                can_error_clear_one();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(g_tx_period_ms));
    }
    vTaskDelete(NULL);
}

static void canman_rx_task(void *arg) {
    (void)arg;
    while (can_running) {
        twai_message_t rx_msg;
        esp_err_t ret = twai_receive(&rx_msg, pdMS_TO_TICKS(1000));
        if (ret == ESP_OK) {
            // Match by identifier and copy into the corresponding frame
            for (size_t i = 0; i < frames_count; ++i) {
                canman_entry_t *e = &frames_registry[i];
                if (e->frame && e->rx_enable && rx_msg.identifier == e->frame->identifier) {
                    xSemaphoreTake(e->m_frame, portMAX_DELAY);
                    *(e->frame) = rx_msg; // struct copy
                    xSemaphoreGive(e->m_frame);
                }
            }
            can_error_clear_one();
        } else {
            ESP_LOGW(TAG, "RX fail: %s", esp_err_to_name(ret));
            can_error_set();
        }
    }
    vTaskDelete(NULL);
}

esp_err_t can_manager_register_frame(twai_message_t *frame, bool tx_enable, bool rx_enable) {
    if (!frame) return ESP_ERR_INVALID_ARG;
    // Registration must happen before init/start to avoid races (no registry mutex)
    if (can_running) return ESP_ERR_INVALID_STATE;
    // Allocate initial registry if needed
    if (!frames_registry || frames_capacity == 0) {
        frames_capacity = 4; // small initial reserve
        frames_registry = (canman_entry_t *)calloc(frames_capacity, sizeof(canman_entry_t));
        if (!frames_registry) return ESP_ERR_NO_MEM;
    }
    // Grow if needed
    if (frames_count >= frames_capacity) {
        size_t new_cap = frames_capacity * 2;
        canman_entry_t *new_mem = (canman_entry_t *)realloc(frames_registry, new_cap * sizeof(canman_entry_t));
        if (!new_mem) return ESP_ERR_NO_MEM;
        memset(new_mem + frames_capacity, 0, (new_cap - frames_capacity) * sizeof(canman_entry_t));
        frames_registry = new_mem;
        frames_capacity = new_cap;
    }
    canman_entry_t *e = &frames_registry[frames_count];
    e->frame = frame;
    e->m_frame = xSemaphoreCreateMutex();
    if (!e->m_frame) {
        return ESP_ERR_NO_MEM;
    }
    e->tx_enable = tx_enable;
    e->rx_enable = rx_enable;
    frames_count++;
    return ESP_OK;
}

esp_err_t can_manager_set_tx_enable(twai_message_t *frame, bool tx_enable) {
    canman_entry_t *e = find_entry_by_frame(frame);
    if (!e) return ESP_ERR_NOT_FOUND;
    e->tx_enable = tx_enable;
    return ESP_OK;
}

esp_err_t can_manager_set_rx_enable(twai_message_t *frame, bool rx_enable) {
    canman_entry_t *e = find_entry_by_frame(frame);
    if (!e) return ESP_ERR_NOT_FOUND;
    e->rx_enable = rx_enable;
    return ESP_OK;
}

esp_err_t can_manager_lock_frame(twai_message_t *frame) {
    canman_entry_t *e = find_entry_by_frame(frame);
    if (!e) return ESP_ERR_NOT_FOUND;
    xSemaphoreTake(e->m_frame, portMAX_DELAY);
    return ESP_OK;
}

esp_err_t can_manager_unlock_frame(twai_message_t *frame) {
    canman_entry_t *e = find_entry_by_frame(frame);
    if (!e) return ESP_ERR_NOT_FOUND;
    xSemaphoreGive(e->m_frame);
    return ESP_OK;
}

bool can_manager_error_get(void) {
    bool v;
    if (m_can_error) xSemaphoreTake(m_can_error, portMAX_DELAY);
    v = can_error;
    if (m_can_error) xSemaphoreGive(m_can_error);
    return v;
}

esp_err_t can_manager_init(const twai_general_config_t *g,
                           const twai_timing_config_t *t,
                           const twai_filter_config_t *f,
                           uint8_t tx_period_ms, const char *tag) {
    if (can_running) return ESP_ERR_INVALID_STATE;
    // Use provided tag for logging; fall back to a default if NULL/empty
    TAG = (tag && tag[0]) ? tag : "CANMAN";

    // No registry mutex; registry must be finalized before init
    if (!m_can_error) m_can_error = xSemaphoreCreateMutex();
    if (!m_can_error) return ESP_ERR_NO_MEM;
    g_tx_period_ms = tx_period_ms;

    // Defaults (mirror current app behavior) if NULLs provided
    ESP_ERROR_CHECK(twai_driver_install(g, t, f));
    esp_err_t res = twai_start();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI: %s", esp_err_to_name(res));
        twai_driver_uninstall();
        return res;
    }
    can_running = true;
    ESP_LOGI(TAG, "TWAI started");

    // Launch tasks
    xTaskCreate(canman_tx_task, "CANMAN_tx", 4096, NULL, 1, NULL);
    xTaskCreate(canman_rx_task, "CANMAN_rx", 4096, NULL, 1, NULL);
    return ESP_OK;
}

esp_err_t can_manager_deinit(void) {
    if (!can_running) return ESP_ERR_INVALID_STATE;
    can_running = false;
    // Give tasks a moment to exit
    vTaskDelay(pdMS_TO_TICKS(20));
    twai_stop();
    twai_driver_uninstall();
    // Note: not deleting mutexes/registry for simplicity; could be added if needed
    return ESP_OK;
}
