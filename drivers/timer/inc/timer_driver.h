#ifndef TIMER_DRIVER_H
#define TIMER_DRIVER_H

#include "driver/gptimer.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Timer configuration structure
 */
typedef struct {
    gptimer_clock_source_t clk_src;       ///< Clock source (GPTIMER_CLK_SRC_APB or GPTIMER_CLK_SRC_XTAL)
    gptimer_count_direction_t direction;  ///< Count direction (GPTIMER_COUNT_UP or GPTIMER_COUNT_DOWN)
    uint64_t resolution_hz;               ///< Timer resolution in Hz (e.g., 1000000 for 1 MHz)
} timer_driver_config_t;

/**
 * @brief Timer alarm configuration structure
 */
typedef struct {
    uint64_t alarm_count;                 ///< Alarm count value
    bool reload_count_on_alarm;           ///< Auto-reload count on alarm (enables periodic alarms)
    gptimer_alarm_cb_t callback;          ///< Callback function (NULL if not used). Signature: bool callback(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*)
    void *user_data;                      ///< User data passed to callback
} timer_driver_alarm_config_t;

/**
 * @brief Initialize timer driver
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t timer_driver_init(void);

/**
 * @brief Create a new timer
 * 
 * @param config Timer configuration
 * @param timer_handle Pointer to store timer handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t timer_driver_create(const timer_driver_config_t *config, gptimer_handle_t *timer_handle);

/**
 * @brief Delete a timer
 * 
 * @param timer_handle Timer handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t timer_driver_delete(gptimer_handle_t timer_handle);

/**
 * @brief Set timer alarm configuration
 * 
 * @param timer_handle Timer handle
 * @param alarm_config Alarm configuration
 * @return esp_err_t ESP_OK on success
 */
esp_err_t timer_driver_set_alarm(gptimer_handle_t timer_handle, const timer_driver_alarm_config_t *alarm_config);

/**
 * @brief Start timer
 * 
 * @param timer_handle Timer handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t timer_driver_start(gptimer_handle_t timer_handle);

/**
 * @brief Stop timer
 * 
 * @param timer_handle Timer handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t timer_driver_stop(gptimer_handle_t timer_handle);

/**
 * @brief Enable timer
 * 
 * @param timer_handle Timer handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t timer_driver_enable(gptimer_handle_t timer_handle);

/**
 * @brief Disable timer
 * 
 * @param timer_handle Timer handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t timer_driver_disable(gptimer_handle_t timer_handle);

/**
 * @brief Set timer count value
 * 
 * @param timer_handle Timer handle
 * @param value Count value to set
 * @return esp_err_t ESP_OK on success
 */
esp_err_t timer_driver_set_raw_count(gptimer_handle_t timer_handle, uint64_t value);

/**
 * @brief Get timer count value
 * 
 * @param timer_handle Timer handle
 * @param value Pointer to store count value
 * @return esp_err_t ESP_OK on success
 */
esp_err_t timer_driver_get_raw_count(gptimer_handle_t timer_handle, uint64_t *value);

/**
 * @brief Deinitialize timer driver
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t timer_driver_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // TIMER_DRIVER_H

