#ifndef DAC_DRIVER_H
#define DAC_DRIVER_H

#include "driver/dac_oneshot.h"
#include "hal/dac_types.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ESP32 has 2 DAC channels
#define DAC_CHANNEL_COUNT 2

/**
 * @brief DAC channel configuration structure
 */
typedef struct {
    dac_channel_t channel;  ///< DAC channel number (DAC_CHAN_0 or DAC_CHAN_1)
} dac_driver_channel_config_t;

/**
 * @brief Initialize DAC driver
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t dac_driver_init(void);

/**
 * @brief Configure and enable DAC channel
 * 
 * @param config DAC channel configuration structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t dac_driver_config_channel(const dac_driver_channel_config_t *config);

/**
 * @brief Set DAC output voltage
 * 
 * @param channel DAC channel number
 * @param voltage_8bit 8-bit voltage value (0-255, where 255 = VDD33)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t dac_driver_set_voltage(dac_channel_t channel, uint8_t voltage_8bit);

/**
 * @brief Get current DAC output voltage
 * 
 * @param channel DAC channel number
 * @return int Current voltage value (0-255), or -1 on error
 */
int dac_driver_get_voltage(dac_channel_t channel);

/**
 * @brief Disable DAC channel
 * 
 * @param channel DAC channel number
 * @return esp_err_t ESP_OK on success
 */
esp_err_t dac_driver_disable_channel(dac_channel_t channel);

/**
 * @brief Enable DAC channel
 * 
 * @param channel DAC channel number
 * @return esp_err_t ESP_OK on success
 */
esp_err_t dac_driver_enable_channel(dac_channel_t channel);

/**
 * @brief Deinitialize DAC driver
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t dac_driver_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // DAC_DRIVER_H
