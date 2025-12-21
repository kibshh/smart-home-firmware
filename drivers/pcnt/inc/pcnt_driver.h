#ifndef PCNT_DRIVER_H
#define PCNT_DRIVER_H

#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum number of PCNT units supported
 */
#define PCNT_DRIVER_MAX_UNITS 4

/**
 * @brief Maximum number of channels per unit
 */
#define PCNT_DRIVER_MAX_CHANNELS_PER_UNIT 2

/**
 * @brief Maximum number of watch points per unit
 */
#define PCNT_DRIVER_MAX_WATCH_POINTS 4

/**
 * @brief PCNT unit configuration structure
 */
typedef struct {
    int low_limit;                  ///< Low limit of counter (minimum: -32768)
    int high_limit;                 ///< High limit of counter (maximum: 32767)
    uint32_t glitch_filter_ns;      ///< Glitch filter in nanoseconds (0 = disabled)
    bool accum_count;               ///< Accumulate count when counter overflows/underflows
} pcnt_driver_unit_config_t;

/**
 * @brief PCNT channel configuration structure
 */
typedef struct {
    gpio_num_t edge_gpio;           ///< GPIO for edge (pulse) signal
    gpio_num_t level_gpio;          ///< GPIO for level (control) signal (-1 if not used)
    pcnt_channel_edge_action_t pos_edge_action;   ///< Action on positive edge (PCNT_CHANNEL_EDGE_ACTION_HOLD/INCREASE/DECREASE)
    pcnt_channel_edge_action_t neg_edge_action;   ///< Action on negative edge
    pcnt_channel_level_action_t high_level_action; ///< Action when level GPIO is high (PCNT_CHANNEL_LEVEL_ACTION_KEEP/INVERSE/HOLD)
    pcnt_channel_level_action_t low_level_action;  ///< Action when level GPIO is low
    bool invert_edge_input;         ///< Invert edge GPIO input
    bool invert_level_input;        ///< Invert level GPIO input
} pcnt_driver_channel_config_t;

/**
 * @brief PCNT watch event data
 */
typedef struct {
    int watch_point_value;          ///< The watch point value that triggered
    int current_count;              ///< Current counter value
    bool zero_cross_detected;       ///< True if zero crossing was detected
} pcnt_driver_watch_event_t;

/**
 * @brief PCNT watch callback function type
 * @param unit_handle Unit handle that triggered the event
 * @param event Event data
 * @param user_data User context data
 * @return true if a high priority task was woken, false otherwise
 */
typedef bool (*pcnt_driver_watch_cb_t)(pcnt_unit_handle_t unit_handle, const pcnt_driver_watch_event_t *event, void *user_data);

/**
 * @brief Initialize PCNT driver
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcnt_driver_init(void);

/**
 * @brief Deinitialize PCNT driver
 * 
 * Forces cleanup of all units and channels.
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcnt_driver_deinit(void);

/**
 * @brief Create a new PCNT unit
 * 
 * @param config Unit configuration
 * @param unit_handle Pointer to store the unit handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcnt_driver_create_unit(const pcnt_driver_unit_config_t *config, pcnt_unit_handle_t *unit_handle);

/**
 * @brief Delete a PCNT unit
 * 
 * Also deletes all channels associated with the unit.
 * 
 * @param unit_handle Unit handle to delete
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcnt_driver_delete_unit(pcnt_unit_handle_t unit_handle);

/**
 * @brief Add a channel to a PCNT unit
 * 
 * @param unit_handle Unit handle
 * @param config Channel configuration
 * @param channel_handle Pointer to store the channel handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcnt_driver_add_channel(pcnt_unit_handle_t unit_handle, const pcnt_driver_channel_config_t *config, pcnt_channel_handle_t *channel_handle);

/**
 * @brief Remove a channel from a PCNT unit
 * 
 * @param channel_handle Channel handle to remove
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcnt_driver_remove_channel(pcnt_channel_handle_t channel_handle);

/**
 * @brief Enable a PCNT unit
 * 
 * Must be called before starting the unit.
 * 
 * @param unit_handle Unit handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcnt_driver_enable(pcnt_unit_handle_t unit_handle);

/**
 * @brief Disable a PCNT unit
 * 
 * @param unit_handle Unit handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcnt_driver_disable(pcnt_unit_handle_t unit_handle);

/**
 * @brief Start counting
 * 
 * Unit must be enabled first.
 * 
 * @param unit_handle Unit handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcnt_driver_start(pcnt_unit_handle_t unit_handle);

/**
 * @brief Stop counting
 * 
 * @param unit_handle Unit handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcnt_driver_stop(pcnt_unit_handle_t unit_handle);

/**
 * @brief Clear the counter value (reset to zero)
 * 
 * @param unit_handle Unit handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcnt_driver_clear_count(pcnt_unit_handle_t unit_handle);

/**
 * @brief Get the current counter value
 * 
 * @param unit_handle Unit handle
 * @param count Pointer to store the count value
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcnt_driver_get_count(pcnt_unit_handle_t unit_handle, int *count);

/**
 * @brief Add a watch point to trigger events
 * 
 * Watch points can trigger callbacks when the counter reaches specific values.
 * 
 * @param unit_handle Unit handle
 * @param watch_point The counter value to watch for
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcnt_driver_add_watch_point(pcnt_unit_handle_t unit_handle, int watch_point);

/**
 * @brief Remove a watch point
 * 
 * @param unit_handle Unit handle
 * @param watch_point The watch point value to remove
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcnt_driver_remove_watch_point(pcnt_unit_handle_t unit_handle, int watch_point);

/**
 * @brief Register a callback for watch point events
 * 
 * @param unit_handle Unit handle
 * @param callback Callback function (NULL to unregister)
 * @param user_data User data to pass to callback
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcnt_driver_register_callback(pcnt_unit_handle_t unit_handle, pcnt_driver_watch_cb_t callback, void *user_data);

/**
 * @brief Set glitch filter for a unit
 * 
 * @param unit_handle Unit handle
 * @param filter_ns Glitch filter in nanoseconds (0 = disabled)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcnt_driver_set_glitch_filter(pcnt_unit_handle_t unit_handle, uint32_t filter_ns);

/**
 * @brief Get the number of active units
 * 
 * @return Number of active PCNT units
 */
uint32_t pcnt_driver_get_active_units(void);

#ifdef __cplusplus
}
#endif

#endif // PCNT_DRIVER_H

