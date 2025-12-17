#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief GPIO configuration structure
 */
typedef struct {
    gpio_num_t pin;              ///< GPIO pin number
    gpio_mode_t mode;            ///< GPIO mode (input/output)
    gpio_pull_mode_t pull_mode;  ///< Pull-up/pull-down configuration
    bool intr_type_enable;       ///< Enable interrupt
    gpio_int_type_t intr_type;   ///< Interrupt type (if enabled)
} gpio_driver_config_t;

/**
 * @brief Initialize GPIO driver
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t gpio_driver_init(void);

/**
 * @brief Configure a GPIO pin
 * 
 * @param config GPIO configuration structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t gpio_driver_config(const gpio_driver_config_t *config);

/**
 * @brief Set GPIO pin level
 * 
 * @param pin GPIO pin number
 * @param level Pin level (0 or 1)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t gpio_driver_set_level(gpio_num_t pin, uint32_t level);

/**
 * @brief Get GPIO pin level
 * 
 * @param pin GPIO pin number
 * @return int Pin level (0 or 1), or -1 on error
 */
int gpio_driver_get_level(gpio_num_t pin);

/**
 * @brief Toggle GPIO pin level
 * 
 * @param pin GPIO pin number
 * @return esp_err_t ESP_OK on success
 */
esp_err_t gpio_driver_toggle(gpio_num_t pin);

/**
 * @brief Install GPIO interrupt service
 * 
 * @param intr_alloc_flags Interrupt allocation flags
 * @return esp_err_t ESP_OK on success
 */
esp_err_t gpio_driver_install_isr(int intr_alloc_flags);

/**
 * @brief Add ISR handler for GPIO pin
 * 
 * @param pin GPIO pin number
 * @param isr_handler ISR handler function
 * @param args Arguments to pass to ISR handler
 * @return esp_err_t ESP_OK on success
 */
esp_err_t gpio_driver_add_isr_handler(gpio_num_t pin, gpio_isr_t isr_handler, void *args);

/**
 * @brief Remove ISR handler for GPIO pin
 * 
 * @param pin GPIO pin number
 * @return esp_err_t ESP_OK on success
 */
esp_err_t gpio_driver_remove_isr_handler(gpio_num_t pin);

/**
 * @brief Deinitialize GPIO driver
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t gpio_driver_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // GPIO_DRIVER_H

