#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "hal/adc_types.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ADC channel configuration structure
 */
typedef struct {
    adc_channel_t channel;       ///< ADC channel number (e.g., ADC_CHANNEL_0)
    adc_atten_t attenuation;     ///< ADC attenuation
} adc_driver_channel_config_t;

/**
 * @brief ADC calibration data structure
 */
typedef struct {
    uint32_t vref;               ///< Reference voltage in mV
    adc_atten_t attenuation;     ///< Attenuation used for calibration
    bool configured;             ///< Whether channel has been configured
    bool calibrated;             ///< Whether calibration data is valid
} adc_driver_cal_data_t;

/**
 * @brief ADC calibration scheme type
 * 
 * Tracks which calibration scheme was used for each channel.
 * This is required to use the correct deletion function when cleaning up
 * calibration handles, as each scheme has its own delete function.
 */
typedef enum {
    ADC_CALI_SCHEME_NONE = 0,           ///< No calibration scheme active
    ADC_CALI_SCHEME_CURVE_FITTING,      ///< Curve fitting calibration scheme
    ADC_CALI_SCHEME_LINE_FITTING,       ///< Line fitting calibration scheme
} adc_cali_scheme_t;

/**
 * @brief Initialize ADC driver
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t adc_driver_init(void);

/**
 * @brief Configure ADC channel
 * 
 * @param config ADC channel configuration structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t adc_driver_config_channel(const adc_driver_channel_config_t *config);

/**
 * @brief Get raw ADC reading
 * 
 * @param channel ADC channel number
 * @return int Raw ADC value (0-4095 for 12-bit), or -1 on error
 */
int adc_driver_get_raw(adc_channel_t channel);

/**
 * @brief Get calibrated voltage reading in millivolts
 * 
 * @param channel ADC channel number
 * @return int Voltage in mV, or -1 on error
 */
int adc_driver_get_voltage(adc_channel_t channel);

/**
 * @brief Calibrate ADC using reference voltage
 * 
 * @param channel ADC channel number
 * @param vref_mv Reference voltage in millivolts (stored for reference but not used)
 *                Note: ESP-IDF calibration schemes (curve/line fitting) use eFuse values
 *                or defaults internally. This parameter is stored but ignored during calibration.
 * @return esp_err_t ESP_OK on success
 */
esp_err_t adc_driver_calibrate(adc_channel_t channel, uint32_t vref_mv);

/**
 * @brief Get calibration data for a channel
 * 
 * @param channel ADC channel number
 * @param cal_data Pointer to store calibration data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t adc_driver_get_calibration(adc_channel_t channel, adc_driver_cal_data_t *cal_data);

/**
 * @brief Set ADC width
 * 
 * @param width ADC bit width
 * @return esp_err_t ESP_OK on success
 */
esp_err_t adc_driver_set_width(adc_bitwidth_t width);

/**
 * @brief Set ADC attenuation
 * 
 * @param channel ADC channel number
 * @param attenuation ADC attenuation
 * @return esp_err_t ESP_OK on success
 */
esp_err_t adc_driver_set_attenuation(adc_channel_t channel, adc_atten_t attenuation);

/**
 * @brief Deinitialize ADC driver
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t adc_driver_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // ADC_DRIVER_H
