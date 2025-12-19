#include "pwm_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "PWM_DRIVER";

// Driver-wide state
static bool driver_initialized = false;
static bool timers_configured[LEDC_TIMER_MAX][LEDC_SPEED_MODE_MAX] = {false};
static bool channels_configured[LEDC_CHANNEL_MAX][LEDC_SPEED_MODE_MAX] = {false};
// Store timer resolution for each timer/speed mode combination
static ledc_timer_bit_t timer_resolution[LEDC_TIMER_MAX][LEDC_SPEED_MODE_MAX] = {0};
// Store timer number for each channel/speed mode combination
static ledc_timer_t channel_timer[LEDC_CHANNEL_MAX][LEDC_SPEED_MODE_MAX] = {0};
// Store last duty cycle before stop (for non-destructive stop/start)
static uint32_t last_duty[LEDC_CHANNEL_MAX][LEDC_SPEED_MODE_MAX] = {0};
static bool channel_stopped[LEDC_CHANNEL_MAX][LEDC_SPEED_MODE_MAX] = {false};
// Track fade status per channel
static bool fade_running[LEDC_CHANNEL_MAX][LEDC_SPEED_MODE_MAX] = {false};
static uint32_t fade_start_time[LEDC_CHANNEL_MAX][LEDC_SPEED_MODE_MAX] = {0};
static uint32_t fade_duration_ms[LEDC_CHANNEL_MAX][LEDC_SPEED_MODE_MAX] = {0};
// Mutex only protects driver-wide state (initialization flag and configuration tracking)
// Per-channel operations rely on ESP-IDF's LEDC API thread safety
static SemaphoreHandle_t driver_mutex = NULL;

esp_err_t pwm_driver_init(void)
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
        ESP_LOGW(TAG, "PWM driver already initialized");
        return ESP_OK;
    }

    // Initialize LEDC service
    esp_err_t ret = ledc_fade_func_install(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        xSemaphoreGive(driver_mutex);
        ESP_LOGE(TAG, "Failed to install LEDC fade service: %s", esp_err_to_name(ret));
        return ret;
    }

    memset(timers_configured, 0, sizeof(timers_configured));
    memset(channels_configured, 0, sizeof(channels_configured));

    driver_initialized = true;
    xSemaphoreGive(driver_mutex);

    ESP_LOGI(TAG, "PWM driver initialized");
    return ESP_OK;
}

esp_err_t pwm_driver_config_timer(const pwm_driver_timer_config_t *config)
{
    // Check initialization state (read-only, no mutex needed)
    if (!driver_initialized) {
        ESP_LOGE(TAG, "PWM driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid configuration pointer");
        return ESP_ERR_INVALID_ARG;
    }

    if (config->timer_num >= LEDC_TIMER_MAX || config->speed_mode >= LEDC_SPEED_MODE_MAX) {
        ESP_LOGE(TAG, "Invalid timer or speed mode");
        return ESP_ERR_INVALID_ARG;
    }

    // ESP-IDF's ledc_timer_config() is thread-safe
    ledc_timer_config_t timer_config = {
        .speed_mode = config->speed_mode,
        .timer_num = config->timer_num,
        .duty_resolution = config->duty_resolution,
        .freq_hz = config->freq_hz,
        .clk_cfg = config->clk_cfg,
    };

    esp_err_t ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure timer (timer=%d, mode=%d): %s", 
                 config->timer_num, config->speed_mode, esp_err_to_name(ret));
        return ret;
    }

    // Track timer configuration and store resolution
    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        timers_configured[config->timer_num][config->speed_mode] = true;
        timer_resolution[config->timer_num][config->speed_mode] = config->duty_resolution;
        xSemaphoreGive(driver_mutex);
    }

    ESP_LOGI(TAG, "PWM timer configured (timer=%d, mode=%d, freq=%lu Hz, resolution=%d bits)",
             config->timer_num, config->speed_mode, (unsigned long)config->freq_hz, 
             (int)config->duty_resolution);
    return ESP_OK;
}

esp_err_t pwm_driver_config_channel(const pwm_driver_channel_config_t *config)
{
    // Check initialization state (read-only, no mutex needed)
    if (!driver_initialized) {
        ESP_LOGE(TAG, "PWM driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid configuration pointer");
        return ESP_ERR_INVALID_ARG;
    }

    if (config->channel >= LEDC_CHANNEL_MAX || config->speed_mode >= LEDC_SPEED_MODE_MAX) {
        ESP_LOGE(TAG, "Invalid channel or speed mode");
        return ESP_ERR_INVALID_ARG;
    }

    // ESP-IDF's ledc_channel_config() is thread-safe
    ledc_channel_config_t channel_config = {
        .speed_mode = config->speed_mode,
        .channel = config->channel,
        .timer_sel = config->timer_num,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = config->gpio_num,
        .duty = config->duty,
        .hpoint = config->hpoint,
    };

    esp_err_t ret = ledc_channel_config(&channel_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure channel (channel=%d, gpio=%d): %s",
                 config->channel, config->gpio_num, esp_err_to_name(ret));
        return ret;
    }

    // Note: Output inversion is not directly supported in all ESP-IDF versions
    // If inversion is needed, configure GPIO inversion separately or use hardware inversion
    if (config->invert) {
        ESP_LOGW(TAG, "Channel inversion requested (channel=%d) - may need GPIO-level configuration",
                 config->channel);
    }

    // Track channel configuration and store timer number
    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        channels_configured[config->channel][config->speed_mode] = true;
        channel_timer[config->channel][config->speed_mode] = config->timer_num;
        xSemaphoreGive(driver_mutex);
    }

    ESP_LOGI(TAG, "PWM channel configured (channel=%d, gpio=%d, timer=%d, duty=%lu)",
             config->channel, config->gpio_num, config->timer_num, (unsigned long)config->duty);
    return ESP_OK;
}

esp_err_t pwm_driver_set_duty(ledc_mode_t speed_mode, ledc_channel_t channel, uint32_t duty)
{
    // Check initialization state (read-only, no mutex needed)
    if (!driver_initialized) {
        ESP_LOGE(TAG, "PWM driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (channel >= LEDC_CHANNEL_MAX || speed_mode >= LEDC_SPEED_MODE_MAX) {
        ESP_LOGE(TAG, "Invalid channel or speed mode");
        return ESP_ERR_INVALID_ARG;
    }

    // ESP-IDF's ledc_set_duty() is thread-safe per channel
    esp_err_t ret = ledc_set_duty(speed_mode, channel, duty);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set duty (channel=%d): %s", channel, esp_err_to_name(ret));
        return ret;
    }

    ret = ledc_update_duty(speed_mode, channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update duty (channel=%d): %s", channel, esp_err_to_name(ret));
        return ret;
    }

    // Update stored duty cycle and mark channel as running (if it was stopped)
    // Also clear fade status if duty is set directly (fade is interrupted)
    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        last_duty[channel][speed_mode] = duty;
        channel_stopped[channel][speed_mode] = false;  // Channel is now running
        fade_running[channel][speed_mode] = false;      // Direct duty set interrupts fade
        xSemaphoreGive(driver_mutex);
    }

    ESP_LOGD(TAG, "PWM duty set (channel=%d, duty=%lu)", channel, (unsigned long)duty);
    return ESP_OK;
}

esp_err_t pwm_driver_get_duty(ledc_mode_t speed_mode, ledc_channel_t channel, uint32_t *duty)
{
    // Check initialization state (read-only, no mutex needed)
    if (!driver_initialized) {
        ESP_LOGE(TAG, "PWM driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (channel >= LEDC_CHANNEL_MAX || speed_mode >= LEDC_SPEED_MODE_MAX || duty == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    // ESP-IDF's ledc_get_duty() is thread-safe per channel
    *duty = ledc_get_duty(speed_mode, channel);
    return ESP_OK;
}

esp_err_t pwm_driver_set_duty_percent(ledc_mode_t speed_mode, ledc_channel_t channel, float percentage)
{
    if (percentage < 0.0f || percentage > 100.0f) {
        ESP_LOGE(TAG, "Invalid percentage (must be 0-100)");
        return ESP_ERR_INVALID_ARG;
    }

    if (channel >= LEDC_CHANNEL_MAX || speed_mode >= LEDC_SPEED_MODE_MAX) {
        ESP_LOGE(TAG, "Invalid channel or speed mode");
        return ESP_ERR_INVALID_ARG;
    }

    // Get timer number and resolution for this channel
    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    if (!channels_configured[channel][speed_mode]) {
        xSemaphoreGive(driver_mutex);
        ESP_LOGE(TAG, "Channel %d not configured for speed mode %d", channel, speed_mode);
        return ESP_ERR_INVALID_STATE;
    }

    ledc_timer_t timer_num = channel_timer[channel][speed_mode];

    if (!timers_configured[timer_num][speed_mode]) {
        xSemaphoreGive(driver_mutex);
        ESP_LOGE(TAG, "Timer %d not configured for speed mode %d", timer_num, speed_mode);
        return ESP_ERR_INVALID_STATE;
    }

    ledc_timer_bit_t resolution = timer_resolution[timer_num][speed_mode];
    xSemaphoreGive(driver_mutex);

    // Calculate max duty based on actual timer resolution
    uint32_t max_duty = (1U << (int)resolution) - 1;
    uint32_t duty = (uint32_t)roundf((percentage / 100.0f) * max_duty);

    return pwm_driver_set_duty(speed_mode, channel, duty);
}

esp_err_t pwm_driver_get_duty_percent(ledc_mode_t speed_mode, ledc_channel_t channel, float *percentage)
{
    if (percentage == NULL) {
        ESP_LOGE(TAG, "Invalid percentage pointer");
        return ESP_ERR_INVALID_ARG;
    }

    if (channel >= LEDC_CHANNEL_MAX || speed_mode >= LEDC_SPEED_MODE_MAX) {
        ESP_LOGE(TAG, "Invalid channel or speed mode");
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t duty;
    esp_err_t ret = pwm_driver_get_duty(speed_mode, channel, &duty);
    if (ret != ESP_OK) {
        return ret;
    }

    // Get timer number and resolution for this channel
    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    if (!channels_configured[channel][speed_mode]) {
        xSemaphoreGive(driver_mutex);
        ESP_LOGE(TAG, "Channel %d not configured for speed mode %d", channel, speed_mode);
        return ESP_ERR_INVALID_STATE;
    }

    ledc_timer_t timer_num = channel_timer[channel][speed_mode];

    if (!timers_configured[timer_num][speed_mode]) {
        xSemaphoreGive(driver_mutex);
        ESP_LOGE(TAG, "Timer %d not configured for speed mode %d", timer_num, speed_mode);
        return ESP_ERR_INVALID_STATE;
    }

    ledc_timer_bit_t resolution = timer_resolution[timer_num][speed_mode];
    xSemaphoreGive(driver_mutex);

    // Calculate percentage based on actual timer resolution
    uint32_t max_duty = (1U << (int)resolution) - 1;
    *percentage = (duty * 100.0f) / max_duty;

    return ESP_OK;
}

esp_err_t pwm_driver_set_freq(ledc_mode_t speed_mode, ledc_timer_t timer_num, uint32_t freq_hz)
{
    // Check initialization state (read-only, no mutex needed)
    if (!driver_initialized) {
        ESP_LOGE(TAG, "PWM driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (timer_num >= LEDC_TIMER_MAX || speed_mode >= LEDC_SPEED_MODE_MAX) {
        ESP_LOGE(TAG, "Invalid timer or speed mode");
        return ESP_ERR_INVALID_ARG;
    }

    // ESP-IDF's ledc_set_freq() is thread-safe per timer
    esp_err_t ret = ledc_set_freq(speed_mode, timer_num, freq_hz);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set frequency (timer=%d, freq=%lu Hz): %s",
                 timer_num, (unsigned long)freq_hz, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "PWM frequency set (timer=%d, freq=%lu Hz)", timer_num, (unsigned long)freq_hz);
    return ESP_OK;
}

esp_err_t pwm_driver_get_freq(ledc_mode_t speed_mode, ledc_timer_t timer_num, uint32_t *freq_hz)
{
    // Check initialization state (read-only, no mutex needed)
    if (!driver_initialized) {
        ESP_LOGE(TAG, "PWM driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (timer_num >= LEDC_TIMER_MAX || speed_mode >= LEDC_SPEED_MODE_MAX || freq_hz == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    // ESP-IDF's ledc_get_freq() is thread-safe per timer
    *freq_hz = ledc_get_freq(speed_mode, timer_num);
    return ESP_OK;
}

esp_err_t pwm_driver_start(ledc_mode_t speed_mode, ledc_channel_t channel)
{
    // Check initialization state (read-only, no mutex needed)
    if (!driver_initialized) {
        ESP_LOGE(TAG, "PWM driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (channel >= LEDC_CHANNEL_MAX || speed_mode >= LEDC_SPEED_MODE_MAX) {
        ESP_LOGE(TAG, "Invalid channel or speed mode");
        return ESP_ERR_INVALID_ARG;
    }

    // Check if channel was stopped and restore last duty cycle
    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if (channel_stopped[channel][speed_mode]) {
            // Restore the last duty cycle before starting
            uint32_t saved_duty = last_duty[channel][speed_mode];
            xSemaphoreGive(driver_mutex);
            
            // Set the saved duty cycle
            esp_err_t ret = pwm_driver_set_duty(speed_mode, channel, saved_duty);
            if (ret != ESP_OK) {
                return ret;
            }
            
            // Mark channel as started
            if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                channel_stopped[channel][speed_mode] = false;
                xSemaphoreGive(driver_mutex);
            }
            
            ESP_LOGD(TAG, "PWM started (channel=%d) with restored duty=%lu", channel, (unsigned long)saved_duty);
            return ESP_OK;
        }
        xSemaphoreGive(driver_mutex);
    }

    // ESP-IDF's ledc_update_duty() starts PWM output (uses current duty cycle)
    esp_err_t ret = ledc_update_duty(speed_mode, channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start PWM (channel=%d): %s", channel, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "PWM started (channel=%d)", channel);
    return ESP_OK;
}

esp_err_t pwm_driver_stop(ledc_mode_t speed_mode, ledc_channel_t channel)
{
    // Check initialization state (read-only, no mutex needed)
    if (!driver_initialized) {
        ESP_LOGE(TAG, "PWM driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (channel >= LEDC_CHANNEL_MAX || speed_mode >= LEDC_SPEED_MODE_MAX) {
        ESP_LOGE(TAG, "Invalid channel or speed mode");
        return ESP_ERR_INVALID_ARG;
    }

    // Save current duty cycle before stopping (non-destructive stop)
    uint32_t current_duty = ledc_get_duty(speed_mode, channel);
    
    // Set duty to 0 to stop PWM output
    esp_err_t ret = pwm_driver_set_duty(speed_mode, channel, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    // Store the last duty cycle and mark channel as stopped
    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        last_duty[channel][speed_mode] = current_duty;
        channel_stopped[channel][speed_mode] = true;
        xSemaphoreGive(driver_mutex);
    }

    ESP_LOGD(TAG, "PWM stopped (channel=%d), saved duty=%lu", channel, (unsigned long)current_duty);
    return ESP_OK;
}

esp_err_t pwm_driver_fade(ledc_mode_t speed_mode, ledc_channel_t channel, uint32_t target_duty, uint32_t fade_time_ms)
{
    // Check initialization state (read-only, no mutex needed)
    if (!driver_initialized) {
        ESP_LOGE(TAG, "PWM driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (channel >= LEDC_CHANNEL_MAX || speed_mode >= LEDC_SPEED_MODE_MAX) {
        ESP_LOGE(TAG, "Invalid channel or speed mode");
        return ESP_ERR_INVALID_ARG;
    }

    // ESP-IDF's ledc_set_fade_with_time() is thread-safe per channel
    esp_err_t ret = ledc_set_fade_with_time(speed_mode, channel, target_duty, fade_time_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set fade (channel=%d): %s", channel, esp_err_to_name(ret));
        return ret;
    }

    ret = ledc_fade_start(speed_mode, channel, LEDC_FADE_NO_WAIT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start fade (channel=%d): %s", channel, esp_err_to_name(ret));
        return ret;
    }

    // Track fade status
    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        fade_running[channel][speed_mode] = true;
        fade_start_time[channel][speed_mode] = xTaskGetTickCount();
        fade_duration_ms[channel][speed_mode] = fade_time_ms;
        xSemaphoreGive(driver_mutex);
    }

    ESP_LOGD(TAG, "PWM fade started (channel=%d, target_duty=%lu, time=%lu ms)",
             channel, (unsigned long)target_duty, (unsigned long)fade_time_ms);
    return ESP_OK;
}

bool pwm_driver_is_fade_running(ledc_mode_t speed_mode, ledc_channel_t channel)
{
    if (!driver_initialized || channel >= LEDC_CHANNEL_MAX || speed_mode >= LEDC_SPEED_MODE_MAX) {
        return false;
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;  // Can't check, assume not running
    }

    bool is_running = fade_running[channel][speed_mode];
    
    // If fade is marked as running, check if fade duration has elapsed
    if (is_running) {
        uint32_t current_time = xTaskGetTickCount();
        uint32_t elapsed_ms = (current_time - fade_start_time[channel][speed_mode]) * portTICK_PERIOD_MS;
        
        if (elapsed_ms >= fade_duration_ms[channel][speed_mode]) {
            // Fade should be complete, clear the flag
            fade_running[channel][speed_mode] = false;
            is_running = false;
        }
    }
    
    xSemaphoreGive(driver_mutex);
    return is_running;
}

esp_err_t pwm_driver_deinit(void)
{
    if (!driver_initialized) {
        ESP_LOGW(TAG, "PWM driver not initialized");
        return ESP_OK;
    }

    if (driver_mutex != NULL) {
        if (xSemaphoreTake(driver_mutex, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to take mutex during deinit");
            return ESP_ERR_TIMEOUT;
        }
    }

    // Check for active channels/timers
    uint32_t active_channels = 0;
    for (int ch = 0; ch < LEDC_CHANNEL_MAX; ch++) {
        for (int mode = 0; mode < LEDC_SPEED_MODE_MAX; mode++) {
            if (channels_configured[ch][mode]) {
                active_channels++;
            }
        }
    }

    if (active_channels > 0) {
        ESP_LOGW(TAG, "Deinitializing driver with %lu active channel(s). Channels should be stopped first.",
                 (unsigned long)active_channels);
    }

    // Uninstall fade service
    ledc_fade_func_uninstall();

    driver_initialized = false;
    memset(timers_configured, 0, sizeof(timers_configured));
    memset(channels_configured, 0, sizeof(channels_configured));
    memset(timer_resolution, 0, sizeof(timer_resolution));
    memset(channel_timer, 0, sizeof(channel_timer));
    memset(last_duty, 0, sizeof(last_duty));
    memset(channel_stopped, 0, sizeof(channel_stopped));
    memset(fade_running, 0, sizeof(fade_running));
    memset(fade_start_time, 0, sizeof(fade_start_time));
    memset(fade_duration_ms, 0, sizeof(fade_duration_ms));

    // Give mutex back before deleting it for clarity
    if (driver_mutex != NULL) {
        xSemaphoreGive(driver_mutex);
        vSemaphoreDelete(driver_mutex);
        driver_mutex = NULL;
    }

    ESP_LOGI(TAG, "PWM driver deinitialized");
    return ESP_OK;
}

