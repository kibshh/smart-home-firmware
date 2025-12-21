#include "pcnt_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "PCNT_DRIVER";

// Per-unit tracking structure
typedef struct {
    pcnt_unit_handle_t handle;
    pcnt_channel_handle_t channels[PCNT_DRIVER_MAX_CHANNELS_PER_UNIT];
    uint32_t channel_count;
    int watch_points[PCNT_DRIVER_MAX_WATCH_POINTS];
    uint32_t watch_point_count;
    pcnt_driver_watch_cb_t user_callback;
    void *user_data;
    bool enabled;
    bool running;
} pcnt_unit_info_t;

// Driver state
static struct {
    bool initialized;
    SemaphoreHandle_t mutex;
    pcnt_unit_info_t units[PCNT_DRIVER_MAX_UNITS];
    uint32_t active_unit_count;
} driver_state = {0};

// Forward declaration
static bool pcnt_internal_callback(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);

// Find unit info by handle (must be called with mutex held)
static pcnt_unit_info_t *find_unit_info(pcnt_unit_handle_t unit_handle)
{
    for (int i = 0; i < PCNT_DRIVER_MAX_UNITS; i++) {
        if (driver_state.units[i].handle == unit_handle) {
            return &driver_state.units[i];
        }
    }
    return NULL;
}

// Find free unit slot (must be called with mutex held)
static pcnt_unit_info_t *find_free_unit_slot(void)
{
    for (int i = 0; i < PCNT_DRIVER_MAX_UNITS; i++) {
        if (driver_state.units[i].handle == NULL) {
            return &driver_state.units[i];
        }
    }
    return NULL;
}

// Find unit info by channel handle (must be called with mutex held)
static pcnt_unit_info_t *find_unit_by_channel(pcnt_channel_handle_t channel_handle, int *channel_index)
{
    for (int i = 0; i < PCNT_DRIVER_MAX_UNITS; i++) {
        if (driver_state.units[i].handle != NULL) {
            for (int j = 0; j < PCNT_DRIVER_MAX_CHANNELS_PER_UNIT; j++) {
                if (driver_state.units[i].channels[j] == channel_handle) {
                    if (channel_index != NULL) {
                        *channel_index = j;
                    }
                    return &driver_state.units[i];
                }
            }
        }
    }
    return NULL;
}

// Internal callback that wraps user callback
static bool pcnt_internal_callback(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    pcnt_unit_info_t *unit_info = (pcnt_unit_info_t *)user_ctx;
    
    if (unit_info == NULL || unit_info->user_callback == NULL) {
        return false;
    }
    
    // Convert ESP-IDF event data to our event structure
    pcnt_driver_watch_event_t event = {
        .watch_point_value = edata->watch_point_value,
        .current_count = 0,  // ESP-IDF doesn't provide this in callback
        .zero_cross_detected = edata->zero_cross_mode,
    };
    
    return unit_info->user_callback(unit, &event, unit_info->user_data);
}

esp_err_t pcnt_driver_init(void)
{
    if (driver_state.initialized) {
        ESP_LOGW(TAG, "PCNT driver already initialized");
        return ESP_OK;
    }
    
    // Create mutex
    driver_state.mutex = xSemaphoreCreateMutex();
    if (driver_state.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize unit tracking
    memset(driver_state.units, 0, sizeof(driver_state.units));
    driver_state.active_unit_count = 0;
    driver_state.initialized = true;
    
    ESP_LOGI(TAG, "PCNT driver initialized");
    return ESP_OK;
}

esp_err_t pcnt_driver_deinit(void)
{
    if (!driver_state.initialized) {
        ESP_LOGW(TAG, "PCNT driver not initialized");
        return ESP_OK;
    }
    
    if (xSemaphoreTake(driver_state.mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex during deinit");
        return ESP_ERR_TIMEOUT;
    }
    
    // Force cleanup of all units
    for (int i = 0; i < PCNT_DRIVER_MAX_UNITS; i++) {
        pcnt_unit_info_t *unit_info = &driver_state.units[i];
        if (unit_info->handle != NULL) {
            ESP_LOGW(TAG, "Force deleting unit %d during deinit", i);
            
            // Stop and disable if running
            if (unit_info->running) {
                pcnt_unit_stop(unit_info->handle);
            }
            if (unit_info->enabled) {
                pcnt_unit_disable(unit_info->handle);
            }
            
            // Delete all channels
            for (int j = 0; j < PCNT_DRIVER_MAX_CHANNELS_PER_UNIT; j++) {
                if (unit_info->channels[j] != NULL) {
                    pcnt_del_channel(unit_info->channels[j]);
                    unit_info->channels[j] = NULL;
                }
            }
            
            // Remove watch points
            for (uint32_t w = 0; w < unit_info->watch_point_count; w++) {
                pcnt_unit_remove_watch_point(unit_info->handle, unit_info->watch_points[w]);
            }
            
            // Delete unit
            pcnt_del_unit(unit_info->handle);
            unit_info->handle = NULL;
        }
    }
    
    driver_state.active_unit_count = 0;
    driver_state.initialized = false;
    
    xSemaphoreGive(driver_state.mutex);
    vSemaphoreDelete(driver_state.mutex);
    driver_state.mutex = NULL;
    
    ESP_LOGI(TAG, "PCNT driver deinitialized");
    return ESP_OK;
}

esp_err_t pcnt_driver_create_unit(const pcnt_driver_unit_config_t *config, pcnt_unit_handle_t *unit_handle)
{
    if (!driver_state.initialized) {
        ESP_LOGE(TAG, "PCNT driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (config == NULL || unit_handle == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Validate limits
    if (config->low_limit < -32768 || config->high_limit > 32767) {
        ESP_LOGE(TAG, "Counter limits out of range (must be -32768 to 32767)");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (config->low_limit >= config->high_limit) {
        ESP_LOGE(TAG, "Low limit must be less than high limit");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(driver_state.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // Find free slot
    pcnt_unit_info_t *unit_info = find_free_unit_slot();
    if (unit_info == NULL) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "No free unit slots (max: %d)", PCNT_DRIVER_MAX_UNITS);
        return ESP_ERR_NO_MEM;
    }
    
    // Configure PCNT unit
    pcnt_unit_config_t unit_config = {
        .low_limit = config->low_limit,
        .high_limit = config->high_limit,
        .flags = {
            .accum_count = config->accum_count,
        },
    };
    
    esp_err_t ret = pcnt_new_unit(&unit_config, unit_handle);
    if (ret != ESP_OK) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Failed to create PCNT unit: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Apply glitch filter if specified
    if (config->glitch_filter_ns > 0) {
        pcnt_glitch_filter_config_t filter_config = {
            .max_glitch_ns = config->glitch_filter_ns,
        };
        ret = pcnt_unit_set_glitch_filter(*unit_handle, &filter_config);
        if (ret != ESP_OK) {
            pcnt_del_unit(*unit_handle);
            xSemaphoreGive(driver_state.mutex);
            ESP_LOGE(TAG, "Failed to set glitch filter: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    
    // Initialize unit tracking
    memset(unit_info, 0, sizeof(pcnt_unit_info_t));
    unit_info->handle = *unit_handle;
    driver_state.active_unit_count++;
    
    xSemaphoreGive(driver_state.mutex);
    
    ESP_LOGI(TAG, "PCNT unit created (limits: %d to %d, filter: %lu ns)",
             config->low_limit, config->high_limit, (unsigned long)config->glitch_filter_ns);
    return ESP_OK;
}

esp_err_t pcnt_driver_delete_unit(pcnt_unit_handle_t unit_handle)
{
    if (!driver_state.initialized) {
        ESP_LOGE(TAG, "PCNT driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (unit_handle == NULL) {
        ESP_LOGE(TAG, "Invalid unit handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(driver_state.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    pcnt_unit_info_t *unit_info = find_unit_info(unit_handle);
    if (unit_info == NULL) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Unit not found");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Stop and disable if running
    if (unit_info->running) {
        pcnt_unit_stop(unit_handle);
        unit_info->running = false;
    }
    if (unit_info->enabled) {
        pcnt_unit_disable(unit_handle);
        unit_info->enabled = false;
    }
    
    // Delete all channels first
    for (int i = 0; i < PCNT_DRIVER_MAX_CHANNELS_PER_UNIT; i++) {
        if (unit_info->channels[i] != NULL) {
            esp_err_t ret = pcnt_del_channel(unit_info->channels[i]);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to delete channel %d: %s", i, esp_err_to_name(ret));
            }
            unit_info->channels[i] = NULL;
        }
    }
    unit_info->channel_count = 0;
    
    // Remove all watch points
    for (uint32_t i = 0; i < unit_info->watch_point_count; i++) {
        pcnt_unit_remove_watch_point(unit_handle, unit_info->watch_points[i]);
    }
    unit_info->watch_point_count = 0;
    
    // Delete the unit
    esp_err_t ret = pcnt_del_unit(unit_handle);
    if (ret != ESP_OK) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Failed to delete PCNT unit: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Clear tracking
    unit_info->handle = NULL;
    unit_info->user_callback = NULL;
    unit_info->user_data = NULL;
    
    if (driver_state.active_unit_count > 0) {
        driver_state.active_unit_count--;
    }
    
    xSemaphoreGive(driver_state.mutex);
    
    ESP_LOGD(TAG, "PCNT unit deleted");
    return ESP_OK;
}

esp_err_t pcnt_driver_add_channel(pcnt_unit_handle_t unit_handle, const pcnt_driver_channel_config_t *config, pcnt_channel_handle_t *channel_handle)
{
    if (!driver_state.initialized) {
        ESP_LOGE(TAG, "PCNT driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (unit_handle == NULL || config == NULL || channel_handle == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!GPIO_IS_VALID_GPIO(config->edge_gpio)) {
        ESP_LOGE(TAG, "Invalid edge GPIO: %d", config->edge_gpio);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (config->level_gpio != -1 && !GPIO_IS_VALID_GPIO(config->level_gpio)) {
        ESP_LOGE(TAG, "Invalid level GPIO: %d", config->level_gpio);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(driver_state.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    pcnt_unit_info_t *unit_info = find_unit_info(unit_handle);
    if (unit_info == NULL) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Unit not found");
        return ESP_ERR_NOT_FOUND;
    }
    
    if (unit_info->channel_count >= PCNT_DRIVER_MAX_CHANNELS_PER_UNIT) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Maximum channels reached for this unit (max: %d)", PCNT_DRIVER_MAX_CHANNELS_PER_UNIT);
        return ESP_ERR_NO_MEM;
    }
    
    // Find free channel slot
    int free_slot = -1;
    for (int i = 0; i < PCNT_DRIVER_MAX_CHANNELS_PER_UNIT; i++) {
        if (unit_info->channels[i] == NULL) {
            free_slot = i;
            break;
        }
    }
    
    if (free_slot < 0) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "No free channel slots");
        return ESP_ERR_NO_MEM;
    }
    
    // Configure channel
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = config->edge_gpio,
        .level_gpio_num = config->level_gpio,
        .flags = {
            .invert_edge_input = config->invert_edge_input,
            .invert_level_input = config->invert_level_input,
        },
    };
    
    esp_err_t ret = pcnt_new_channel(unit_handle, &chan_config, channel_handle);
    if (ret != ESP_OK) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Failed to create PCNT channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set edge actions
    ret = pcnt_channel_set_edge_action(*channel_handle, config->pos_edge_action, config->neg_edge_action);
    if (ret != ESP_OK) {
        pcnt_del_channel(*channel_handle);
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Failed to set edge actions: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set level actions
    ret = pcnt_channel_set_level_action(*channel_handle, config->high_level_action, config->low_level_action);
    if (ret != ESP_OK) {
        pcnt_del_channel(*channel_handle);
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Failed to set level actions: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Store channel handle
    unit_info->channels[free_slot] = *channel_handle;
    unit_info->channel_count++;
    
    xSemaphoreGive(driver_state.mutex);
    
    ESP_LOGI(TAG, "PCNT channel added (edge GPIO: %d, level GPIO: %d)",
             config->edge_gpio, config->level_gpio);
    return ESP_OK;
}

esp_err_t pcnt_driver_remove_channel(pcnt_channel_handle_t channel_handle)
{
    if (!driver_state.initialized) {
        ESP_LOGE(TAG, "PCNT driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (channel_handle == NULL) {
        ESP_LOGE(TAG, "Invalid channel handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(driver_state.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    int channel_index = -1;
    pcnt_unit_info_t *unit_info = find_unit_by_channel(channel_handle, &channel_index);
    if (unit_info == NULL || channel_index < 0) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Channel not found");
        return ESP_ERR_NOT_FOUND;
    }
    
    esp_err_t ret = pcnt_del_channel(channel_handle);
    if (ret != ESP_OK) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Failed to delete PCNT channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    unit_info->channels[channel_index] = NULL;
    if (unit_info->channel_count > 0) {
        unit_info->channel_count--;
    }
    
    xSemaphoreGive(driver_state.mutex);
    
    ESP_LOGD(TAG, "PCNT channel removed");
    return ESP_OK;
}

esp_err_t pcnt_driver_enable(pcnt_unit_handle_t unit_handle)
{
    if (!driver_state.initialized) {
        ESP_LOGE(TAG, "PCNT driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (unit_handle == NULL) {
        ESP_LOGE(TAG, "Invalid unit handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(driver_state.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    pcnt_unit_info_t *unit_info = find_unit_info(unit_handle);
    if (unit_info == NULL) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Unit not found");
        return ESP_ERR_NOT_FOUND;
    }
    
    if (unit_info->enabled) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGW(TAG, "Unit already enabled");
        return ESP_OK;
    }
    
    esp_err_t ret = pcnt_unit_enable(unit_handle);
    if (ret != ESP_OK) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Failed to enable PCNT unit: %s", esp_err_to_name(ret));
        return ret;
    }
    
    unit_info->enabled = true;
    
    xSemaphoreGive(driver_state.mutex);
    
    ESP_LOGD(TAG, "PCNT unit enabled");
    return ESP_OK;
}

esp_err_t pcnt_driver_disable(pcnt_unit_handle_t unit_handle)
{
    if (!driver_state.initialized) {
        ESP_LOGE(TAG, "PCNT driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (unit_handle == NULL) {
        ESP_LOGE(TAG, "Invalid unit handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(driver_state.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    pcnt_unit_info_t *unit_info = find_unit_info(unit_handle);
    if (unit_info == NULL) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Unit not found");
        return ESP_ERR_NOT_FOUND;
    }
    
    if (!unit_info->enabled) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGW(TAG, "Unit already disabled");
        return ESP_OK;
    }
    
    // Stop first if running
    if (unit_info->running) {
        pcnt_unit_stop(unit_handle);
        unit_info->running = false;
    }
    
    esp_err_t ret = pcnt_unit_disable(unit_handle);
    if (ret != ESP_OK) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Failed to disable PCNT unit: %s", esp_err_to_name(ret));
        return ret;
    }
    
    unit_info->enabled = false;
    
    xSemaphoreGive(driver_state.mutex);
    
    ESP_LOGD(TAG, "PCNT unit disabled");
    return ESP_OK;
}

esp_err_t pcnt_driver_start(pcnt_unit_handle_t unit_handle)
{
    if (!driver_state.initialized) {
        ESP_LOGE(TAG, "PCNT driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (unit_handle == NULL) {
        ESP_LOGE(TAG, "Invalid unit handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(driver_state.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    pcnt_unit_info_t *unit_info = find_unit_info(unit_handle);
    if (unit_info == NULL) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Unit not found");
        return ESP_ERR_NOT_FOUND;
    }
    
    if (!unit_info->enabled) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Unit must be enabled before starting");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (unit_info->running) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGW(TAG, "Unit already running");
        return ESP_OK;
    }
    
    esp_err_t ret = pcnt_unit_start(unit_handle);
    if (ret != ESP_OK) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Failed to start PCNT unit: %s", esp_err_to_name(ret));
        return ret;
    }
    
    unit_info->running = true;
    
    xSemaphoreGive(driver_state.mutex);
    
    ESP_LOGD(TAG, "PCNT unit started");
    return ESP_OK;
}

esp_err_t pcnt_driver_stop(pcnt_unit_handle_t unit_handle)
{
    if (!driver_state.initialized) {
        ESP_LOGE(TAG, "PCNT driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (unit_handle == NULL) {
        ESP_LOGE(TAG, "Invalid unit handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(driver_state.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    pcnt_unit_info_t *unit_info = find_unit_info(unit_handle);
    if (unit_info == NULL) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Unit not found");
        return ESP_ERR_NOT_FOUND;
    }
    
    if (!unit_info->running) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGW(TAG, "Unit already stopped");
        return ESP_OK;
    }
    
    esp_err_t ret = pcnt_unit_stop(unit_handle);
    if (ret != ESP_OK) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Failed to stop PCNT unit: %s", esp_err_to_name(ret));
        return ret;
    }
    
    unit_info->running = false;
    
    xSemaphoreGive(driver_state.mutex);
    
    ESP_LOGD(TAG, "PCNT unit stopped");
    return ESP_OK;
}

esp_err_t pcnt_driver_clear_count(pcnt_unit_handle_t unit_handle)
{
    if (!driver_state.initialized) {
        ESP_LOGE(TAG, "PCNT driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (unit_handle == NULL) {
        ESP_LOGE(TAG, "Invalid unit handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Take mutex to prevent unit deletion during this call
    if (xSemaphoreTake(driver_state.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // Verify unit still exists
    if (find_unit_info(unit_handle) == NULL) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Unit not found (may have been deleted)");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = pcnt_unit_clear_count(unit_handle);
    
    xSemaphoreGive(driver_state.mutex);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear PCNT count: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGD(TAG, "PCNT count cleared");
    return ESP_OK;
}

esp_err_t pcnt_driver_get_count(pcnt_unit_handle_t unit_handle, int *count)
{
    if (!driver_state.initialized) {
        ESP_LOGE(TAG, "PCNT driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (unit_handle == NULL || count == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Take mutex to prevent unit deletion during this call
    if (xSemaphoreTake(driver_state.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // Verify unit still exists
    if (find_unit_info(unit_handle) == NULL) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Unit not found (may have been deleted)");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = pcnt_unit_get_count(unit_handle, count);
    
    xSemaphoreGive(driver_state.mutex);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get PCNT count: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t pcnt_driver_add_watch_point(pcnt_unit_handle_t unit_handle, int watch_point)
{
    if (!driver_state.initialized) {
        ESP_LOGE(TAG, "PCNT driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (unit_handle == NULL) {
        ESP_LOGE(TAG, "Invalid unit handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(driver_state.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    pcnt_unit_info_t *unit_info = find_unit_info(unit_handle);
    if (unit_info == NULL) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Unit not found");
        return ESP_ERR_NOT_FOUND;
    }
    
    if (unit_info->watch_point_count >= PCNT_DRIVER_MAX_WATCH_POINTS) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Maximum watch points reached (max: %d)", PCNT_DRIVER_MAX_WATCH_POINTS);
        return ESP_ERR_NO_MEM;
    }
    
    // Check for duplicate
    for (uint32_t i = 0; i < unit_info->watch_point_count; i++) {
        if (unit_info->watch_points[i] == watch_point) {
            xSemaphoreGive(driver_state.mutex);
            ESP_LOGW(TAG, "Watch point %d already exists", watch_point);
            return ESP_OK;
        }
    }
    
    esp_err_t ret = pcnt_unit_add_watch_point(unit_handle, watch_point);
    if (ret != ESP_OK) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Failed to add watch point %d: %s", watch_point, esp_err_to_name(ret));
        return ret;
    }
    
    unit_info->watch_points[unit_info->watch_point_count] = watch_point;
    unit_info->watch_point_count++;
    
    xSemaphoreGive(driver_state.mutex);
    
    ESP_LOGD(TAG, "Watch point %d added", watch_point);
    return ESP_OK;
}

esp_err_t pcnt_driver_remove_watch_point(pcnt_unit_handle_t unit_handle, int watch_point)
{
    if (!driver_state.initialized) {
        ESP_LOGE(TAG, "PCNT driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (unit_handle == NULL) {
        ESP_LOGE(TAG, "Invalid unit handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(driver_state.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    pcnt_unit_info_t *unit_info = find_unit_info(unit_handle);
    if (unit_info == NULL) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Unit not found");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Find watch point
    int found_index = -1;
    for (uint32_t i = 0; i < unit_info->watch_point_count; i++) {
        if (unit_info->watch_points[i] == watch_point) {
            found_index = (int)i;
            break;
        }
    }
    
    if (found_index < 0) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGW(TAG, "Watch point %d not found", watch_point);
        return ESP_ERR_NOT_FOUND;
    }
    
    esp_err_t ret = pcnt_unit_remove_watch_point(unit_handle, watch_point);
    if (ret != ESP_OK) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Failed to remove watch point %d: %s", watch_point, esp_err_to_name(ret));
        return ret;
    }
    
    // Remove from array by shifting
    for (uint32_t i = (uint32_t)found_index; i < unit_info->watch_point_count - 1; i++) {
        unit_info->watch_points[i] = unit_info->watch_points[i + 1];
    }
    unit_info->watch_point_count--;
    
    xSemaphoreGive(driver_state.mutex);
    
    ESP_LOGD(TAG, "Watch point %d removed", watch_point);
    return ESP_OK;
}

esp_err_t pcnt_driver_register_callback(pcnt_unit_handle_t unit_handle, pcnt_driver_watch_cb_t callback, void *user_data)
{
    if (!driver_state.initialized) {
        ESP_LOGE(TAG, "PCNT driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (unit_handle == NULL) {
        ESP_LOGE(TAG, "Invalid unit handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(driver_state.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    pcnt_unit_info_t *unit_info = find_unit_info(unit_handle);
    if (unit_info == NULL) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Unit not found");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Store user callback and data
    unit_info->user_callback = callback;
    unit_info->user_data = user_data;
    
    // Register internal callback (or unregister if callback is NULL)
    pcnt_event_callbacks_t cbs = {
        .on_reach = (callback != NULL) ? pcnt_internal_callback : NULL,
    };
    
    esp_err_t ret = pcnt_unit_register_event_callbacks(unit_handle, &cbs, unit_info);
    if (ret != ESP_OK) {
        unit_info->user_callback = NULL;
        unit_info->user_data = NULL;
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Failed to register callbacks: %s", esp_err_to_name(ret));
        return ret;
    }
    
    xSemaphoreGive(driver_state.mutex);
    
    ESP_LOGD(TAG, "Callback %s", callback != NULL ? "registered" : "unregistered");
    return ESP_OK;
}

esp_err_t pcnt_driver_set_glitch_filter(pcnt_unit_handle_t unit_handle, uint32_t filter_ns)
{
    if (!driver_state.initialized) {
        ESP_LOGE(TAG, "PCNT driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (unit_handle == NULL) {
        ESP_LOGE(TAG, "Invalid unit handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(driver_state.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    pcnt_unit_info_t *unit_info = find_unit_info(unit_handle);
    if (unit_info == NULL) {
        xSemaphoreGive(driver_state.mutex);
        ESP_LOGE(TAG, "Unit not found");
        return ESP_ERR_NOT_FOUND;
    }
    
    esp_err_t ret;
    if (filter_ns > 0) {
        pcnt_glitch_filter_config_t filter_config = {
            .max_glitch_ns = filter_ns,
        };
        ret = pcnt_unit_set_glitch_filter(unit_handle, &filter_config);
    } else {
        // Disable filter by passing NULL
        ret = pcnt_unit_set_glitch_filter(unit_handle, NULL);
    }
    
    xSemaphoreGive(driver_state.mutex);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set glitch filter: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGD(TAG, "Glitch filter set to %lu ns", (unsigned long)filter_ns);
    return ESP_OK;
}

uint32_t pcnt_driver_get_active_units(void)
{
    if (!driver_state.initialized) {
        return 0;
    }
    
    uint32_t count = 0;
    if (xSemaphoreTake(driver_state.mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        count = driver_state.active_unit_count;
        xSemaphoreGive(driver_state.mutex);
    }
    
    return count;
}

