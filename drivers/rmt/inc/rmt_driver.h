#ifndef RMT_DRIVER_H
#define RMT_DRIVER_H

#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum number of RMT channels
 */
#define RMT_CHANNEL_MAX 8

/**
 * @brief Maximum number of symbols in pre-allocated buffer
 */
#define RMT_MAX_SYMBOLS_PER_TRANSACTION 512

/**
 * @brief RMT driver context structure (opaque)
 */
typedef struct rmt_driver_context rmt_driver_context_t;

/**
 * @brief RMT transmit configuration structure
 */
typedef struct {
    gpio_num_t gpio_num;              ///< GPIO pin for RMT TX
    rmt_clock_source_t clk_src;        ///< Clock source (RMT_CLK_SRC_DEFAULT, RMT_CLK_SRC_APB, RMT_CLK_SRC_REF_TICK, RMT_CLK_SRC_XTAL)
    uint32_t resolution_hz;            ///< RMT resolution in Hz (e.g., 1000000 for 1 MHz)
    uint32_t carrier_freq_hz;          ///< Carrier frequency in Hz (0 to disable carrier, typically 38000 for IR)
    float carrier_duty_percent;        ///< Carrier duty cycle percentage (0-100, typically 33% for IR)
    bool carrier_en;                   ///< Enable carrier modulation
    bool carrier_level;                ///< Carrier level (true = high, false = low)
    bool loop_en;                      ///< Enable loop transmission
    uint32_t loop_count;              ///< Number of loops (0 = infinite, only valid if loop_en = true)
    uint32_t mem_block_symbols;       ///< Memory block size in symbols (0 = auto)
    uint32_t trans_queue_depth;        ///< Transmission queue depth (default: 4)
} rmt_driver_tx_config_t;

/**
 * @brief RMT receive configuration structure
 */
typedef struct {
    gpio_num_t gpio_num;              ///< GPIO pin for RMT RX
    rmt_clock_source_t clk_src;        ///< Clock source (RMT_CLK_SRC_DEFAULT, RMT_CLK_SRC_APB, RMT_CLK_SRC_REF_TICK, RMT_CLK_SRC_XTAL)
    uint32_t resolution_hz;          ///< RMT resolution in Hz (e.g., 1000000 for 1 MHz)
    uint32_t filter_ticks;            ///< Filter ticks (0 to disable filter, typically 100 for IR)
    bool filter_en;                    ///< Enable filter
    uint32_t threshold;                ///< Signal threshold (minimum duration to be considered a pause/gap, in ticks)
    uint32_t mem_block_symbols;       ///< Memory block size in symbols (0 = auto)
    uint32_t event_queue_depth;       ///< Event queue depth for RX completion events (default: 4)
} rmt_driver_rx_config_t;

/**
 * @brief RMT symbol structure (for pattern-based transmission)
 */
typedef struct {
    uint32_t level0;                   ///< Level 0 value (0 or 1)
    uint32_t duration0;                ///< Level 0 duration in ticks
    uint32_t level1;                   ///< Level 1 value (0 or 1)
    uint32_t duration1;                ///< Level 1 duration in ticks
} rmt_symbol_t;

/**
 * @brief Initialize RMT driver and create context
 * 
 * @param context Pointer to store driver context (NULL for global instance)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rmt_driver_init(rmt_driver_context_t **context);

/**
 * @brief Initialize RMT driver (global instance, for backward compatibility)
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rmt_driver_init_global(void);

/**
 * @brief Configure RMT channel for transmission
 * 
 * @param context Driver context (NULL for global instance)
 * @param channel RMT channel number (0-7)
 * @param config TX configuration structure
 * @param tx_handle Pointer to store TX handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rmt_driver_config_tx(rmt_driver_context_t *context, uint32_t channel, const rmt_driver_tx_config_t *config, rmt_channel_handle_t *tx_handle);

/**
 * @brief Configure RMT channel for reception
 * 
 * @param context Driver context (NULL for global instance)
 * @param channel RMT channel number (0-7)
 * @param config RX configuration structure
 * @param rx_handle Pointer to store RX handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rmt_driver_config_rx(rmt_driver_context_t *context, uint32_t channel, const rmt_driver_rx_config_t *config, rmt_channel_handle_t *rx_handle);

/**
 * @brief Transmit RMT symbols (pattern-based)
 * 
 * @param context Driver context (NULL for global instance)
 * @param tx_handle TX channel handle
 * @param symbols Array of RMT symbols
 * @param num_symbols Number of symbols in array
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rmt_driver_transmit(rmt_driver_context_t *context, rmt_channel_handle_t tx_handle, const rmt_symbol_t *symbols, size_t num_symbols, uint32_t timeout_ms);

/**
 * @brief Transmit RMT symbols with wait (blocking)
 * 
 * @param context Driver context (NULL for global instance)
 * @param tx_handle TX channel handle
 * @param symbols Array of RMT symbols
 * @param num_symbols Number of symbols in array
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rmt_driver_transmit_wait(rmt_driver_context_t *context, rmt_channel_handle_t tx_handle, const rmt_symbol_t *symbols, size_t num_symbols, uint32_t timeout_ms);

/**
 * @brief Receive RMT symbols
 * 
 * @param context Driver context (NULL for global instance)
 * @param rx_handle RX channel handle
 * @param symbols Buffer to store received symbols
 * @param num_symbols Maximum number of symbols to receive
 * @param received_symbols Pointer to store actual number of symbols received
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT on timeout
 */
esp_err_t rmt_driver_receive(rmt_driver_context_t *context, rmt_channel_handle_t rx_handle, rmt_symbol_t *symbols, size_t num_symbols, size_t *received_symbols, uint32_t timeout_ms);

/**
 * @brief Enable RMT channel
 * 
 * @param context Driver context (NULL for global instance)
 * @param handle Channel handle (TX or RX)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rmt_driver_enable(rmt_driver_context_t *context, rmt_channel_handle_t handle);

/**
 * @brief Disable RMT channel
 * 
 * @param context Driver context (NULL for global instance)
 * @param handle Channel handle (TX or RX)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rmt_driver_disable(rmt_driver_context_t *context, rmt_channel_handle_t handle);

/**
 * @brief Delete RMT TX channel
 * 
 * @param context Driver context (NULL for global instance)
 * @param tx_handle TX channel handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rmt_driver_delete_tx(rmt_driver_context_t *context, rmt_channel_handle_t tx_handle);

/**
 * @brief Delete RMT RX channel
 * 
 * @param context Driver context (NULL for global instance)
 * @param rx_handle RX channel handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rmt_driver_delete_rx(rmt_driver_context_t *context, rmt_channel_handle_t rx_handle);

/**
 * @brief Deinitialize RMT driver (global instance)
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rmt_driver_deinit_global(void);

/**
 * @brief Deinitialize RMT driver context
 * 
 * @param context Driver context to deinitialize (NULL for global instance)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rmt_driver_deinit_context(rmt_driver_context_t *context);

#ifdef __cplusplus
}
#endif

#endif // RMT_DRIVER_H

