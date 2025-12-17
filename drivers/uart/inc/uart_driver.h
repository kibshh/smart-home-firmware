#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UART_PORT_COUNT 3  // ESP32 has UART_NUM_0, UART_NUM_1, UART_NUM_2

/**
 * @brief UART configuration structure
 */
typedef struct {
    uart_port_t port_num;              ///< UART port number (UART_NUM_0, UART_NUM_1, or UART_NUM_2)
    int baud_rate;                     ///< Baud rate (e.g., 115200, 9600)
    uart_word_length_t data_bits;      ///< Data bits (UART_DATA_5_BITS to UART_DATA_8_BITS)
    uart_parity_t parity;              ///< Parity (UART_PARITY_DISABLE, UART_PARITY_EVEN, UART_PARITY_ODD)
    uart_stop_bits_t stop_bits;        ///< Stop bits (UART_STOP_BITS_1, UART_STOP_BITS_1_5, UART_STOP_BITS_2)
    uart_hw_flowcontrol_t flow_ctrl;  ///< Hardware flow control (UART_HW_FLOWCTRL_DISABLE, UART_HW_FLOWCTRL_RTS, UART_HW_FLOWCTRL_CTS, UART_HW_FLOWCTRL_CTS_RTS)
    int tx_io_num;                     ///< TX GPIO pin number
    int rx_io_num;                     ///< RX GPIO pin number
    int rts_io_num;                    ///< RTS GPIO pin number (-1 if not used)
    int cts_io_num;                    ///< CTS GPIO pin number (-1 if not used)
    int rx_buffer_size;                ///< RX buffer size in bytes
    int tx_buffer_size;                ///< TX buffer size in bytes
    int queue_size;                    ///< UART event queue size (0 = no event queue, >0 = required for pattern detection)
    int intr_alloc_flags;              ///< Interrupt allocation flags
    uart_sclk_t source_clk;            ///< UART clock source: UART_SCLK_APB (default, normal operation), UART_SCLK_REF_TICK (low power, dynamic baud accuracy)
} uart_driver_config_t;

/**
 * @brief Initialize UART driver
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t uart_driver_init(void);

/**
 * @brief Configure UART port
 * 
 * @param config UART configuration structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t uart_driver_config(const uart_driver_config_t *config);

/**
 * @brief Deconfigure UART port
 * 
 * @param port_num UART port number
 * @return esp_err_t ESP_OK on success
 */
esp_err_t uart_driver_deconfig(uart_port_t port_num);

/**
 * @brief Write data to UART port
 * 
 * @param port_num UART port number
 * @param data Pointer to data buffer
 * @param len Length of data to write
 * @param timeout_ms Timeout in milliseconds (0 = no timeout, portMAX_DELAY = wait forever)
 * @return int Number of bytes written, or negative on error
 */
int uart_driver_write(uart_port_t port_num, const uint8_t *data, size_t len, int timeout_ms);

/**
 * @brief Read data from UART port
 * 
 * @param port_num UART port number
 * @param data Pointer to data buffer
 * @param len Maximum length to read
 * @param timeout_ms Timeout in milliseconds (0 = no timeout, portMAX_DELAY = wait forever)
 * @return int Number of bytes read, or negative on error
 */
int uart_driver_read(uart_port_t port_num, uint8_t *data, size_t len, int timeout_ms);

/**
 * @brief Get number of bytes available in RX buffer
 * 
 * @param port_num UART port number
 * @return int Number of bytes available, or negative on error
 */
int uart_driver_get_buffered_data_len(uart_port_t port_num);

/**
 * @brief Flush RX buffer
 * 
 * @param port_num UART port number
 * @return esp_err_t ESP_OK on success
 */
esp_err_t uart_driver_flush_rx(uart_port_t port_num);

/**
 * @brief Flush TX buffer
 * 
 * @param port_num UART port number
 * @return esp_err_t ESP_OK on success
 */
esp_err_t uart_driver_flush_tx(uart_port_t port_num);

/**
 * @brief Set baud rate for UART port
 * 
 * @param port_num UART port number
 * @param baud_rate Baud rate
 * @return esp_err_t ESP_OK on success
 */
esp_err_t uart_driver_set_baudrate(uart_port_t port_num, uint32_t baud_rate);

/**
 * @brief Get baud rate for UART port
 * 
 * @param port_num UART port number
 * @param baud_rate Pointer to store baud rate
 * @return esp_err_t ESP_OK on success
 */
esp_err_t uart_driver_get_baudrate(uart_port_t port_num, uint32_t *baud_rate);

/**
 * @brief Set pattern detection for UART RX
 * 
 * @note Pattern detection requires an event queue. Set queue_size > 0 in uart_driver_config()
 *       before calling this function.
 * 
 * @param port_num UART port number
 * @param pattern_chr Character to detect
 * @param chr_num Number of pattern characters to detect
 * @param chr_tout Timeout between pattern characters (in number of UART bit times)
 * @param post_idle Idle time after pattern detection (in number of UART bit times)
 * @param pre_idle Idle time before pattern detection (in number of UART bit times)
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if event queue not configured
 */
esp_err_t uart_driver_set_pattern(uart_port_t port_num, char pattern_chr, uint8_t chr_num, int chr_tout, int post_idle, int pre_idle);

/**
 * @brief Wait for and get pattern detection event from queue
 * 
 * @note Pattern detection requires an event queue. Set queue_size > 0 in uart_driver_config()
 *       before using pattern detection.
 * 
 * @param port_num UART port number
 * @param timeout_ms Timeout in milliseconds (0 = no wait, <0 = wait forever)
 * @return esp_err_t ESP_OK on success (pattern detected), ESP_ERR_TIMEOUT if no pattern event received
 */
esp_err_t uart_driver_get_pattern(uart_port_t port_num, int timeout_ms);

/**
 * @brief Check if UART port is configured
 * 
 * @param port_num UART port number
 * @return true if configured, false otherwise
 */
bool uart_driver_is_configured(uart_port_t port_num);

/**
 * @brief Deinitialize UART driver
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t uart_driver_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // UART_DRIVER_H

