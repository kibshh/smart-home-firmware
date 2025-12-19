#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum number of PWM channels per timer
 */
#define PWM_MAX_CHANNELS_PER_TIMER 8

/**
 * @brief PWM timer configuration structure
 */
typedef struct {
    ledc_timer_t timer_num;          ///< Timer number (LEDC_TIMER_0 to LEDC_TIMER_3)
    ledc_mode_t speed_mode;          ///< Speed mode (LEDC_LOW_SPEED_MODE or LEDC_HIGH_SPEED_MODE)
    ledc_timer_bit_t duty_resolution; ///< Duty resolution in bits (e.g., LEDC_TIMER_13_BIT for 13-bit)
    uint32_t freq_hz;                ///< PWM frequency in Hz
    ledc_clk_cfg_t clk_cfg;          ///< Clock source configuration
} pwm_driver_timer_config_t;

/**
 * @brief PWM channel configuration structure
 */
typedef struct {
    ledc_timer_t timer_num;          ///< Timer to use (must match timer config)
    ledc_channel_t channel;          ///< Channel number (LEDC_CHANNEL_0 to LEDC_CHANNEL_7)
    gpio_num_t gpio_num;             ///< GPIO pin number
    ledc_mode_t speed_mode;          ///< Speed mode (must match timer config)
    uint32_t duty;                   ///< Initial duty cycle (0 to max_duty based on resolution)
    uint32_t hpoint;                 ///< Phase point (0 for no phase shift)
    bool invert;                     ///< Invert output signal
} pwm_driver_channel_config_t;

/**
 * @brief Initialize PWM driver
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pwm_driver_init(void);

/**
 * @brief Configure PWM timer
 * 
 * @param config Timer configuration
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pwm_driver_config_timer(const pwm_driver_timer_config_t *config);

/**
 * @brief Configure PWM channel
 * 
 * @param config Channel configuration
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pwm_driver_config_channel(const pwm_driver_channel_config_t *config);

/**
 * @brief Set PWM duty cycle
 * 
 * @param speed_mode Speed mode
 * @param channel Channel number
 * @param duty Duty cycle value (0 to max_duty based on timer resolution)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pwm_driver_set_duty(ledc_mode_t speed_mode, ledc_channel_t channel, uint32_t duty);

/**
 * @brief Get PWM duty cycle
 * 
 * @param speed_mode Speed mode
 * @param channel Channel number
 * @param duty Pointer to store duty cycle value
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pwm_driver_get_duty(ledc_mode_t speed_mode, ledc_channel_t channel, uint32_t *duty);

/**
 * @brief Set PWM duty cycle in percentage (0-100%)
 * 
 * @param speed_mode Speed mode
 * @param channel Channel number
 * @param percentage Duty cycle percentage (0.0 to 100.0)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pwm_driver_set_duty_percent(ledc_mode_t speed_mode, ledc_channel_t channel, float percentage);

/**
 * @brief Get PWM duty cycle in percentage (0-100%)
 * 
 * @param speed_mode Speed mode
 * @param channel Channel number
 * @param percentage Pointer to store duty cycle percentage
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pwm_driver_get_duty_percent(ledc_mode_t speed_mode, ledc_channel_t channel, float *percentage);

/**
 * @brief Set PWM frequency
 * 
 * @param speed_mode Speed mode
 * @param timer_num Timer number
 * @param freq_hz Frequency in Hz
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pwm_driver_set_freq(ledc_mode_t speed_mode, ledc_timer_t timer_num, uint32_t freq_hz);

/**
 * @brief Get PWM frequency
 * 
 * @param speed_mode Speed mode
 * @param timer_num Timer number
 * @param freq_hz Pointer to store frequency in Hz
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pwm_driver_get_freq(ledc_mode_t speed_mode, ledc_timer_t timer_num, uint32_t *freq_hz);

/**
 * @brief Start PWM output on channel
 * 
 * @param speed_mode Speed mode
 * @param channel Channel number
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pwm_driver_start(ledc_mode_t speed_mode, ledc_channel_t channel);

/**
 * @brief Stop PWM output on channel
 * 
 * @param speed_mode Speed mode
 * @param channel Channel number
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pwm_driver_stop(ledc_mode_t speed_mode, ledc_channel_t channel);

/**
 * @brief Fade PWM duty cycle over time
 * 
 * @param speed_mode Speed mode
 * @param channel Channel number
 * @param target_duty Target duty cycle
 * @param fade_time_ms Fade time in milliseconds
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pwm_driver_fade(ledc_mode_t speed_mode, ledc_channel_t channel, uint32_t target_duty, uint32_t fade_time_ms);

/**
 * @brief Check if fade is in progress
 * 
 * @param speed_mode Speed mode
 * @param channel Channel number
 * @return true if fade is in progress, false otherwise
 */
bool pwm_driver_is_fade_running(ledc_mode_t speed_mode, ledc_channel_t channel);

/**
 * @brief Deinitialize PWM driver
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pwm_driver_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // PWM_DRIVER_H

