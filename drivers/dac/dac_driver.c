#include "dac_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "DAC_DRIVER";
static bool driver_initialized = false;
static SemaphoreHandle_t driver_mutex = NULL;
static dac_oneshot_handle_t dac_handles[DAC_CHANNEL_COUNT] = {NULL};
static bool channel_enabled[DAC_CHANNEL_COUNT] = {false};
static uint8_t channel_voltage[DAC_CHANNEL_COUNT] = {0};

static int channel_to_index(dac_channel_t channel)
{
    // ESP32 DAC channels: DAC_CHAN_0 = 0, DAC_CHAN_1 = 1
    if (channel == DAC_CHAN_0) {
        return 0;
    } else if (channel == DAC_CHAN_1) {
        return 1;
    }
    return -1;
}

esp_err_t dac_driver_init(void)
{
    if (driver_initialized) {
        ESP_LOGW(TAG, "DAC driver already initialized");
        return ESP_OK;
    }

    // Create mutex for thread-safe operations
    driver_mutex = xSemaphoreCreateMutex();
    if (driver_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize state tracking
    // Channels will be enabled when explicitly configured via dac_driver_config_channel()
    memset(dac_handles, 0, sizeof(dac_handles));
    memset(channel_enabled, 0, sizeof(channel_enabled));
    memset(channel_voltage, 0, sizeof(channel_voltage));

    driver_initialized = true;
    ESP_LOGI(TAG, "DAC driver initialized");
    return ESP_OK;
}

esp_err_t dac_driver_config_channel(const dac_driver_channel_config_t *config)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "DAC driver not initialized. Call dac_driver_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid configuration pointer");
        return ESP_ERR_INVALID_ARG;
    }

    int idx = channel_to_index(config->channel);
    if (idx < 0) {
        ESP_LOGE(TAG, "Invalid DAC channel: %d", config->channel);
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    // If channel already configured, delete old handle first
    if (dac_handles[idx] != NULL) {
        dac_oneshot_del_channel(dac_handles[idx]);
        dac_handles[idx] = NULL;
    }

    // Create new DAC channel handle
    dac_oneshot_config_t dac_config = {
        .chan_id = config->channel,
    };
    esp_err_t ret = dac_oneshot_new_channel(&dac_config, &dac_handles[idx]);
    if (ret != ESP_OK) {
        xSemaphoreGive(driver_mutex);
        ESP_LOGE(TAG, "Failed to create DAC channel %d: %s", config->channel, esp_err_to_name(ret));
        return ret;
    }

    channel_enabled[idx] = true;

    xSemaphoreGive(driver_mutex);

    ESP_LOGI(TAG, "DAC channel %d configured and enabled", config->channel);
    return ESP_OK;
}

esp_err_t dac_driver_set_voltage(dac_channel_t channel, uint8_t voltage_8bit)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "DAC driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    int idx = channel_to_index(channel);
    if (idx < 0) {
        ESP_LOGE(TAG, "Invalid DAC channel: %d", channel);
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    // Check if channel is enabled
    if (!channel_enabled[idx] || dac_handles[idx] == NULL) {
        xSemaphoreGive(driver_mutex);
        ESP_LOGE(TAG, "DAC channel %d is not configured", channel);
        return ESP_ERR_INVALID_STATE;
    }

    // Set DAC output voltage (0-255, where 255 = VDD33)
    esp_err_t ret = dac_oneshot_output_voltage(dac_handles[idx], voltage_8bit);
    if (ret == ESP_OK) {
        channel_voltage[idx] = voltage_8bit;
    }

    xSemaphoreGive(driver_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set DAC voltage: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

int dac_driver_get_voltage(dac_channel_t channel)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "DAC driver not initialized");
        return -1;
    }

    int idx = channel_to_index(channel);
    if (idx < 0) {
        ESP_LOGE(TAG, "Invalid DAC channel: %d", channel);
        return -1;
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return -1;
    }

    int voltage = channel_voltage[idx];

    xSemaphoreGive(driver_mutex);

    return voltage;
}

esp_err_t dac_driver_disable_channel(dac_channel_t channel)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "DAC driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    int idx = channel_to_index(channel);
    if (idx < 0) {
        ESP_LOGE(TAG, "Invalid DAC channel: %d", channel);
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    // Truly disable by deleting the channel handle
    if (dac_handles[idx] != NULL) {
        dac_oneshot_del_channel(dac_handles[idx]);
        dac_handles[idx] = NULL;
        channel_enabled[idx] = false;
    }

    xSemaphoreGive(driver_mutex);

    ESP_LOGI(TAG, "DAC channel %d disabled", channel);
    return ESP_OK;
}

esp_err_t dac_driver_enable_channel(dac_channel_t channel)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "DAC driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    int idx = channel_to_index(channel);
    if (idx < 0) {
        ESP_LOGE(TAG, "Invalid DAC channel: %d", channel);
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    // If handle doesn't exist, create it
    if (dac_handles[idx] == NULL) {
        dac_oneshot_config_t dac_config = {
            .chan_id = channel,
        };
        esp_err_t ret = dac_oneshot_new_channel(&dac_config, &dac_handles[idx]);
        if (ret != ESP_OK) {
            xSemaphoreGive(driver_mutex);
            ESP_LOGE(TAG, "Failed to create DAC channel %d: %s", channel, esp_err_to_name(ret));
            return ret;
        }
    }

    channel_enabled[idx] = true;
    // Restore previous voltage if any
    if (channel_voltage[idx] > 0) {
        dac_oneshot_output_voltage(dac_handles[idx], channel_voltage[idx]);
    }

    xSemaphoreGive(driver_mutex);

    ESP_LOGI(TAG, "DAC channel %d enabled", channel);
    return ESP_OK;
}

esp_err_t dac_driver_deinit(void)
{
    if (!driver_initialized) {
        ESP_LOGW(TAG, "DAC driver not initialized");
        return ESP_OK;
    }

    // Take mutex to prevent race conditions
    if (driver_mutex != NULL) {
        if (xSemaphoreTake(driver_mutex, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to take mutex during deinit");
            return ESP_ERR_TIMEOUT;
        }
    }

    // Delete all DAC channel handles
    for (int i = 0; i < DAC_CHANNEL_COUNT; i++) {
        if (dac_handles[i] != NULL) {
            dac_oneshot_del_channel(dac_handles[i]);
            dac_handles[i] = NULL;
        }
    }

    memset(channel_enabled, 0, sizeof(channel_enabled));
    memset(channel_voltage, 0, sizeof(channel_voltage));

    driver_initialized = false;

    // Delete mutex last
    if (driver_mutex != NULL) {
        vSemaphoreDelete(driver_mutex);
        driver_mutex = NULL;
    }

    ESP_LOGI(TAG, "DAC driver deinitialized");
    return ESP_OK;
}
