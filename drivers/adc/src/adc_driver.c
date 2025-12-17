#include "adc_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>

// Note: This driver assumes adc_channel_t values are sequential 0..N-1 for ADC1.
// This is true for ESP32 but should be verified if porting to other platforms.
_Static_assert(ADC_CHANNEL_0 == 0, "ADC channel enum must start at 0");

static const char *TAG = "ADC_DRIVER";
static bool driver_initialized = false;
static SemaphoreHandle_t driver_mutex = NULL;
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc1_cali_handles[SOC_ADC_CHANNEL_NUM(1)] = {NULL};
static adc_cali_scheme_t cali_schemes[SOC_ADC_CHANNEL_NUM(1)] = {ADC_CALI_SCHEME_NONE};
static adc_driver_cal_data_t cal_data[SOC_ADC_CHANNEL_NUM(1)] = {0};
static adc_bitwidth_t current_width = ADC_BITWIDTH_12;

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle, adc_cali_scheme_t *out_scheme)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;
    adc_cali_scheme_t scheme = ADC_CALI_SCHEME_NONE;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGD(TAG, "Trying calibration scheme: Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = current_width,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
            scheme = ADC_CALI_SCHEME_CURVE_FITTING;
            ESP_LOGI(TAG, "Calibration scheme: Curve Fitting (success)");
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGD(TAG, "Trying calibration scheme: Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = current_width,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
            scheme = ADC_CALI_SCHEME_LINE_FITTING;
            ESP_LOGI(TAG, "Calibration scheme: Line Fitting (success)");
        }
    }
#endif

    *out_handle = handle;
    *out_scheme = scheme;
    return calibrated;
}

static void adc_calibration_delete(adc_channel_t channel)
{
    if (channel >= SOC_ADC_CHANNEL_NUM(1)) {
        return;
    }

    if (adc1_cali_handles[channel] == NULL) {
        return;
    }

    switch (cali_schemes[channel]) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        case ADC_CALI_SCHEME_CURVE_FITTING:
            adc_cali_delete_scheme_curve_fitting(adc1_cali_handles[channel]);
            break;
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        case ADC_CALI_SCHEME_LINE_FITTING:
            adc_cali_delete_scheme_line_fitting(adc1_cali_handles[channel]);
            break;
#endif
        case ADC_CALI_SCHEME_NONE:
        default:
            break;
    }

    adc1_cali_handles[channel] = NULL;
    cali_schemes[channel] = ADC_CALI_SCHEME_NONE;
}

esp_err_t adc_driver_init(void)
{
    if (driver_initialized) {
        ESP_LOGW(TAG, "ADC driver already initialized");
        return ESP_OK;
    }

    // Create mutex for thread-safe operations
    driver_mutex = xSemaphoreCreateMutex();
    if (driver_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize ADC oneshot unit
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        vSemaphoreDelete(driver_mutex);
        return ret;
    }

    // Set default ADC width (12-bit, can be changed later via adc_driver_set_width())
    // Note: Actual channel configuration will be done via adc_driver_config_channel()
    current_width = ADC_BITWIDTH_12;

    // Initialize calibration handles array
    memset(adc1_cali_handles, 0, sizeof(adc1_cali_handles));
    memset(cali_schemes, 0, sizeof(cali_schemes));
    memset(cal_data, 0, sizeof(cal_data));

    driver_initialized = true;
    ESP_LOGI(TAG, "ADC driver initialized");
    return ESP_OK;
}

esp_err_t adc_driver_config_channel(const adc_driver_channel_config_t *config)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "ADC driver not initialized. Call adc_driver_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid configuration pointer");
        return ESP_ERR_INVALID_ARG;
    }

    if (config->channel >= SOC_ADC_CHANNEL_NUM(1)) {
        ESP_LOGE(TAG, "Invalid ADC channel: %d", config->channel);
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    // Configure ADC channel
    adc_oneshot_chan_cfg_t adc_config = {
        .bitwidth = current_width,
        .atten = config->attenuation,
    };
    esp_err_t ret = adc_oneshot_config_channel(adc1_handle, config->channel, &adc_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel %d: %s", config->channel, esp_err_to_name(ret));
        xSemaphoreGive(driver_mutex);
        return ret;
    }

    // Store attenuation and mark channel as configured
    cal_data[config->channel].attenuation = config->attenuation;
    cal_data[config->channel].configured = true;

    xSemaphoreGive(driver_mutex);

    ESP_LOGI(TAG, "ADC channel %d configured successfully", config->channel);
    return ESP_OK;
}

int adc_driver_get_raw(adc_channel_t channel)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "ADC driver not initialized");
        return -1;
    }

    if (channel >= SOC_ADC_CHANNEL_NUM(1)) {
        ESP_LOGE(TAG, "Invalid ADC channel: %d", channel);
        return -1;
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return -1;
    }

    int adc_raw = 0;
    esp_err_t ret = adc_oneshot_read(adc1_handle, channel, &adc_raw);
    
    xSemaphoreGive(driver_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ADC channel %d: %s", channel, esp_err_to_name(ret));
        return -1;
    }

    return adc_raw;
}

int adc_driver_get_voltage(adc_channel_t channel)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "ADC driver not initialized");
        return -1;
    }

    if (channel >= SOC_ADC_CHANNEL_NUM(1)) {
        ESP_LOGE(TAG, "Invalid ADC channel: %d", channel);
        return -1;
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return -1;
    }

    // Check calibration state while holding mutex
    if (!cal_data[channel].calibrated || adc1_cali_handles[channel] == NULL) {
        xSemaphoreGive(driver_mutex);
        ESP_LOGE(TAG, "ADC channel %d not calibrated. Call adc_driver_calibrate() first.", channel);
        return -1;
    }

    // Store calibration handle locally (protected by mutex)
    adc_cali_handle_t cali_handle = adc1_cali_handles[channel];
    
    // Read raw value while holding mutex (adc_oneshot_read is thread-safe but we protect our state)
    int adc_raw = 0;
    esp_err_t ret = adc_oneshot_read(adc1_handle, channel, &adc_raw);
    
    xSemaphoreGive(driver_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ADC channel %d: %s", channel, esp_err_to_name(ret));
        return -1;
    }

    // Convert to voltage using stored handle (adc_cali_raw_to_voltage is thread-safe)
    int voltage = 0;
    ret = adc_cali_raw_to_voltage(cali_handle, adc_raw, &voltage);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to convert raw to voltage for channel %d: %s", channel, esp_err_to_name(ret));
        return -1;
    }

    return voltage;
}

esp_err_t adc_driver_calibrate(adc_channel_t channel, uint32_t vref_mv)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "ADC driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (channel >= SOC_ADC_CHANNEL_NUM(1)) {
        ESP_LOGE(TAG, "Invalid ADC channel: %d", channel);
        return ESP_ERR_INVALID_ARG;
    }

    // Note: vref_mv parameter is stored but not used by ESP-IDF calibration schemes.
    // ESP-IDF calibration (curve/line fitting) uses eFuse values or defaults internally.
    // The vref_mv value is stored for reference only.
    if (vref_mv != 0) {
        ESP_LOGW(TAG, "vref_mv parameter (%lu mV) is stored but not used by ESP-IDF calibration schemes. ESP-IDF uses eFuse values or defaults.", vref_mv);
    }

    // Require channel to be configured before calibration
    if (!cal_data[channel].configured) {
        ESP_LOGE(TAG, "Channel %d not configured. Call adc_driver_config_channel() first.", channel);
        return ESP_ERR_INVALID_STATE;
    }

    adc_atten_t attenuation = cal_data[channel].attenuation;

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    // Clean up existing calibration handle if any
    if (cal_data[channel].calibrated) {
        cal_data[channel].calibrated = false;  // Set before deleting to avoid race window
        adc_calibration_delete(channel);
    }

    // Initialize calibration for this channel
    // Note: ESP-IDF calibration schemes use their own Vref (from eFuse or defaults)
    adc_cali_scheme_t scheme = ADC_CALI_SCHEME_NONE;
    bool calibrated = adc_calibration_init(ADC_UNIT_1, channel, attenuation, &adc1_cali_handles[channel], &scheme);
    if (!calibrated) {
        xSemaphoreGive(driver_mutex);
        ESP_LOGE(TAG, "Failed to initialize calibration for channel %d", channel);
        return ESP_FAIL;
    }

    // Store user-provided Vref for reference (even though ESP-IDF doesn't use it)
    cal_data[channel].vref = vref_mv;
    cal_data[channel].attenuation = attenuation;
    cal_data[channel].calibrated = true;
    cali_schemes[channel] = scheme;

    xSemaphoreGive(driver_mutex);

    ESP_LOGI(TAG, "ADC channel %d calibrated with attenuation %d (ESP-IDF uses internal Vref, user Vref %lu mV stored for reference)", channel, attenuation, vref_mv);
    return ESP_OK;
}

esp_err_t adc_driver_get_calibration(adc_channel_t channel, adc_driver_cal_data_t *cal_data_out)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "ADC driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (channel >= SOC_ADC_CHANNEL_NUM(1)) {
        ESP_LOGE(TAG, "Invalid ADC channel: %d", channel);
        return ESP_ERR_INVALID_ARG;
    }

    if (cal_data_out == NULL) {
        ESP_LOGE(TAG, "Invalid calibration data pointer");
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    memcpy(cal_data_out, &cal_data[channel], sizeof(adc_driver_cal_data_t));
    
    xSemaphoreGive(driver_mutex);
    return ESP_OK;
}

esp_err_t adc_driver_set_width(adc_bitwidth_t width)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "ADC driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    // Update global width and invalidate all calibrations since width changed
    if (current_width != width) {
        current_width = width;
        // Invalidate all calibrations as they depend on width
        for (int i = 0; i < SOC_ADC_CHANNEL_NUM(1); i++) {
            if (cal_data[i].calibrated) {
                cal_data[i].calibrated = false;  // Set before deleting to avoid race window
                adc_calibration_delete(i);
            }
        }
        
        // Reconfigure all configured channels with new width
        for (int i = 0; i < SOC_ADC_CHANNEL_NUM(1); i++) {
            if (cal_data[i].configured) {
                adc_oneshot_chan_cfg_t adc_config = {
                    .bitwidth = current_width,
                    .atten = cal_data[i].attenuation,
                };
                esp_err_t ret = adc_oneshot_config_channel(adc1_handle, i, &adc_config);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to reconfigure channel %d with new width: %s", i, esp_err_to_name(ret));
                }
            }
        }
        
        ESP_LOGW(TAG, "ADC width changed, all calibrations invalidated and channels reconfigured");
    }

    xSemaphoreGive(driver_mutex);

    ESP_LOGI(TAG, "ADC width set to %d", width);
    return ESP_OK;
}

esp_err_t adc_driver_set_attenuation(adc_channel_t channel, adc_atten_t attenuation)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "ADC driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (channel >= SOC_ADC_CHANNEL_NUM(1)) {
        ESP_LOGE(TAG, "Invalid ADC channel: %d", channel);
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    // Reconfigure channel with new attenuation
    adc_oneshot_chan_cfg_t adc_config = {
        .bitwidth = current_width,
        .atten = attenuation,
    };
    esp_err_t ret = adc_oneshot_config_channel(adc1_handle, channel, &adc_config);
    if (ret != ESP_OK) {
        xSemaphoreGive(driver_mutex);
        ESP_LOGE(TAG, "Failed to set ADC attenuation for channel %d: %s", channel, esp_err_to_name(ret));
        return ret;
    }

    // Update stored attenuation and invalidate calibration if attenuation changed
    if (cal_data[channel].attenuation != attenuation) {
        cal_data[channel].attenuation = attenuation;
        // Invalidate calibration for this channel
        if (cal_data[channel].calibrated) {
            cal_data[channel].calibrated = false;  // Set before deleting to avoid race window
            adc_calibration_delete(channel);
        }
        ESP_LOGW(TAG, "ADC channel %d attenuation changed, calibration invalidated", channel);
    }
    cal_data[channel].configured = true;

    xSemaphoreGive(driver_mutex);

    ESP_LOGI(TAG, "ADC channel %d attenuation set", channel);
    return ESP_OK;
}

esp_err_t adc_driver_deinit(void)
{
    if (!driver_initialized) {
        ESP_LOGW(TAG, "ADC driver not initialized");
        return ESP_OK;
    }

    // Take mutex to prevent race conditions with other operations
    // Use portMAX_DELAY to wait indefinitely - deinit should not be called during normal operation
    if (driver_mutex != NULL) {
        if (xSemaphoreTake(driver_mutex, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to take mutex during deinit");
            return ESP_ERR_TIMEOUT;
        }
    }

    // Clean up calibration handles
    for (int i = 0; i < SOC_ADC_CHANNEL_NUM(1); i++) {
        adc_calibration_delete(i);
    }

    // Clean up ADC unit
    if (adc1_handle != NULL) {
        adc_oneshot_del_unit(adc1_handle);
        adc1_handle = NULL;
    }

    memset(cal_data, 0, sizeof(cal_data));
    memset(cali_schemes, 0, sizeof(cali_schemes));

    driver_initialized = false;

    // Delete mutex last, after all cleanup is done
    if (driver_mutex != NULL) {
        vSemaphoreDelete(driver_mutex);
        driver_mutex = NULL;
    }

    ESP_LOGI(TAG, "ADC driver deinitialized");
    return ESP_OK;
}
