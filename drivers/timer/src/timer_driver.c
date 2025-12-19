#include "timer_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "TIMER_DRIVER";

// Driver-wide state
static bool driver_initialized = false;
static uint32_t active_timer_count = 0;  // Track number of active timers
// Mutex only protects driver-wide state (initialization flag and timer count)
// Per-timer operations rely on ESP-IDF's GPTimer API thread safety
static SemaphoreHandle_t driver_mutex = NULL;

esp_err_t timer_driver_init(void)
{
    // Protect driver-wide state (initialization flag) with mutex
    if (driver_mutex == NULL) {
        driver_mutex = xSemaphoreCreateMutex();
        if (driver_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    if (driver_initialized) {
        xSemaphoreGive(driver_mutex);
        ESP_LOGW(TAG, "Timer driver already initialized");
        return ESP_OK;
    }

    driver_initialized = true;
    xSemaphoreGive(driver_mutex);

    ESP_LOGI(TAG, "Timer driver initialized");
    return ESP_OK;
}

esp_err_t timer_driver_create(const timer_driver_config_t *config, gptimer_handle_t *timer_handle)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "Timer driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (config == NULL || timer_handle == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    // ESP-IDF's gptimer_new_timer() is thread-safe, no mutex needed
    gptimer_config_t timer_config = {
        .clk_src = config->clk_src,
        .direction = config->direction,
        .resolution_hz = config->resolution_hz,
    };

    esp_err_t ret = gptimer_new_timer(&timer_config, timer_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Track active timer count
    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        active_timer_count++;
        xSemaphoreGive(driver_mutex);
    }

    ESP_LOGI(TAG, "Timer created successfully (handle=%p, resolution=%llu Hz, active=%lu)", 
             (void*)*timer_handle, config->resolution_hz, (unsigned long)active_timer_count);
    return ESP_OK;
}

esp_err_t timer_driver_delete(gptimer_handle_t timer_handle)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "Timer driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (timer_handle == NULL) {
        ESP_LOGE(TAG, "Invalid timer handle");
        return ESP_ERR_INVALID_ARG;
    }

    // ESP-IDF's gptimer_del_timer() is thread-safe, no mutex needed
    esp_err_t ret = gptimer_del_timer(timer_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete timer (handle=%p): %s", (void*)timer_handle, esp_err_to_name(ret));
        return ret;
    }

    // Update active timer count
    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if (active_timer_count > 0) {
            active_timer_count--;
        }
        xSemaphoreGive(driver_mutex);
    }

    ESP_LOGI(TAG, "Timer deleted successfully (handle=%p, active=%lu)", 
             (void*)timer_handle, (unsigned long)active_timer_count);
    return ESP_OK;
}

esp_err_t timer_driver_set_alarm(gptimer_handle_t timer_handle, const timer_driver_alarm_config_t *alarm_config)
{
    // Check initialization state (read-only, no mutex needed)
    if (!driver_initialized) {
        ESP_LOGE(TAG, "Timer driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (timer_handle == NULL || alarm_config == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    // ESP-IDF's GPTimer operations are thread-safe per timer handle

    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = alarm_config->alarm_count,
    };
    
    // Set auto-reload flag if requested (enables periodic alarms)
    if (alarm_config->reload_count_on_alarm) {
        alarm_cfg.flags.auto_reload_on_alarm = true;
    }

    esp_err_t ret = gptimer_set_alarm_action(timer_handle, &alarm_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set alarm (handle=%p): %s", (void*)timer_handle, esp_err_to_name(ret));
        return ret;
    }

    // Register callback if provided
    if (alarm_config->callback != NULL) {
        gptimer_event_callbacks_t cbs = {
            .on_alarm = alarm_config->callback,
        };
        ret = gptimer_register_event_callbacks(timer_handle, &cbs, alarm_config->user_data);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register callback (handle=%p): %s", (void*)timer_handle, esp_err_to_name(ret));
            return ret;
        }
    }

    ESP_LOGD(TAG, "Alarm configured (handle=%p): count=%llu, reload=%d", 
             (void*)timer_handle, alarm_config->alarm_count, alarm_config->reload_count_on_alarm);
    return ESP_OK;
}

esp_err_t timer_driver_start(gptimer_handle_t timer_handle)
{
    // Check initialization state (read-only, no mutex needed)
    if (!driver_initialized) {
        ESP_LOGE(TAG, "Timer driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (timer_handle == NULL) {
        ESP_LOGE(TAG, "Invalid timer handle");
        return ESP_ERR_INVALID_ARG;
    }

    // ESP-IDF's gptimer_start() is thread-safe per timer handle
    esp_err_t ret = gptimer_start(timer_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start timer (handle=%p): %s", (void*)timer_handle, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "Timer started (handle=%p)", (void*)timer_handle);
    return ESP_OK;
}

esp_err_t timer_driver_stop(gptimer_handle_t timer_handle)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "Timer driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (timer_handle == NULL) {
        ESP_LOGE(TAG, "Invalid timer handle");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = gptimer_stop(timer_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop timer (handle=%p): %s", (void*)timer_handle, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "Timer stopped (handle=%p)", (void*)timer_handle);
    return ESP_OK;
}

esp_err_t timer_driver_enable(gptimer_handle_t timer_handle)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "Timer driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (timer_handle == NULL) {
        ESP_LOGE(TAG, "Invalid timer handle");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = gptimer_enable(timer_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable timer (handle=%p): %s", (void*)timer_handle, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "Timer enabled (handle=%p)", (void*)timer_handle);
    return ESP_OK;
}

esp_err_t timer_driver_disable(gptimer_handle_t timer_handle)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "Timer driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (timer_handle == NULL) {
        ESP_LOGE(TAG, "Invalid timer handle");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = gptimer_disable(timer_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable timer (handle=%p): %s", (void*)timer_handle, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "Timer disabled (handle=%p)", (void*)timer_handle);
    return ESP_OK;
}

esp_err_t timer_driver_set_raw_count(gptimer_handle_t timer_handle, uint64_t value)
{
    // Check initialization state (read-only, no mutex needed)
    if (!driver_initialized) {
        ESP_LOGE(TAG, "Timer driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (timer_handle == NULL) {
        ESP_LOGE(TAG, "Invalid timer handle");
        return ESP_ERR_INVALID_ARG;
    }

    // ESP-IDF's gptimer_set_raw_count() is thread-safe per timer handle
    esp_err_t ret = gptimer_set_raw_count(timer_handle, value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set count (handle=%p): %s", (void*)timer_handle, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "Timer count set (handle=%p): %llu", (void*)timer_handle, value);
    return ESP_OK;
}

esp_err_t timer_driver_get_raw_count(gptimer_handle_t timer_handle, uint64_t *value)
{
    // Check initialization state (read-only, no mutex needed)
    if (!driver_initialized) {
        ESP_LOGE(TAG, "Timer driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (timer_handle == NULL || value == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    // ESP-IDF's gptimer_get_raw_count() is thread-safe per timer handle
    esp_err_t ret = gptimer_get_raw_count(timer_handle, value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get count (handle=%p): %s", (void*)timer_handle, esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t timer_driver_deinit(void)
{
    if (!driver_initialized) {
        ESP_LOGW(TAG, "Timer driver not initialized");
        return ESP_OK;
    }

    if (driver_mutex != NULL) {
        if (xSemaphoreTake(driver_mutex, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to take mutex during deinit");
            return ESP_ERR_TIMEOUT;
        }
    }

    // Check for active timers
    if (active_timer_count > 0) {
        ESP_LOGW(TAG, "Deinitializing driver with %lu active timer(s). Timers should be deleted first.", 
                 (unsigned long)active_timer_count);
        // Continue with deinit anyway, but warn the user
    }

    driver_initialized = false;
    active_timer_count = 0;

    // Give mutex back before deleting it for clarity
    if (driver_mutex != NULL) {
        xSemaphoreGive(driver_mutex);
        vSemaphoreDelete(driver_mutex);
        driver_mutex = NULL;
    }

    ESP_LOGI(TAG, "Timer driver deinitialized");
    return ESP_OK;
}

