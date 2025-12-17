#ifndef SPI_SLAVE_DRIVER_H
#define SPI_SLAVE_DRIVER_H

#include "driver/spi_slave.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SPI_SLAVE_HOST_COUNT 3     // ESP32 has SPI_HOST (SPI1), HSPI_HOST (SPI2), VSPI_HOST (SPI3)

/**
 * @brief Callback function type for slave transaction events
 * 
 * @param trans Pointer to the transaction that triggered the callback
 */
typedef void (*spi_slave_callback_t)(spi_slave_transaction_t *trans);

/**
 * @brief SPI slave configuration structure
 * 
 * @note For DMA transfers:
 *       - Buffers must be in DRAM (not flash)
 *       - 4-byte alignment required
 *       - Buffer size must be multiple of 4 bytes for DMA
 */
typedef struct {
    int host_id;                        ///< SPI host ID (SPI2_HOST or SPI3_HOST, SPI1 reserved for flash)
    int mosi_io_num;                    ///< MOSI GPIO pin number
    int miso_io_num;                    ///< MISO GPIO pin number
    int sclk_io_num;                    ///< SCLK GPIO pin number
    int cs_io_num;                      ///< CS GPIO pin number
    int dma_chan;                       ///< DMA channel (0 = no DMA, 1 or 2 for DMA, SPI_DMA_CH_AUTO for auto)
    int queue_size;                     ///< Transaction queue size (number of transactions that can be queued)
    uint8_t mode;                       ///< SPI mode (0-3)
    uint32_t flags;                     ///< SPI slave flags (SPI_SLAVE_*)
    spi_slave_callback_t post_setup_cb;     ///< Callback called after transaction setup (optional, can be NULL)
    spi_slave_callback_t post_trans_cb;     ///< Callback called after transaction completes (optional, can be NULL)
} spi_slave_driver_config_t;

/**
 * @brief Initialize SPI slave driver
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t spi_slave_driver_init(void);

/**
 * @brief Configure SPI slave on specified host
 * 
 * @note Only one slave can be configured per SPI host.
 *       SPI1_HOST is reserved for flash and cannot be used.
 * 
 * @param config SPI slave configuration
 * @return esp_err_t ESP_OK on success
 */
esp_err_t spi_slave_driver_config(const spi_slave_driver_config_t *config);

/**
 * @brief Deinitialize SPI slave on specified host
 * 
 * @param host_id SPI host ID
 * @return esp_err_t ESP_OK on success
 */
esp_err_t spi_slave_driver_free(int host_id);

/**
 * @brief Queue a transaction for SPI slave
 * 
 * This function queues TX data to send when master initiates a transaction,
 * and a buffer to receive data from master. The function returns immediately.
 * Use spi_slave_driver_get_result() to wait for completion.
 * 
 * @note Buffers must be in DRAM and 4-byte aligned for DMA.
 *       Buffer sizes should be multiples of 4 bytes for DMA.
 * 
 * @param host_id SPI host ID
 * @param tx_data Data to send to master (can be NULL)
 * @param rx_data Buffer to receive data from master (can be NULL)
 * @param len Length in bytes (must match expected transaction size)
 * @param timeout_ms Timeout to queue the transaction (ms)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t spi_slave_driver_queue_trans(int host_id, const uint8_t *tx_data, uint8_t *rx_data, size_t len, int timeout_ms);

/**
 * @brief Wait for a slave transaction to complete
 * 
 * @param host_id SPI host ID
 * @param timeout_ms Timeout to wait for transaction completion (ms)
 * @param out_trans_len Pointer to store actual transaction length (can be NULL)
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if no transaction completed in time
 */
esp_err_t spi_slave_driver_get_result(int host_id, int timeout_ms, size_t *out_trans_len);

/**
 * @brief Perform a blocking slave transaction
 * 
 * Queues TX/RX data and waits for master to complete the transaction.
 * This is a convenience function combining queue and get_result.
 * 
 * @param host_id SPI host ID
 * @param tx_data Data to send to master (can be NULL)
 * @param rx_data Buffer to receive data from master (can be NULL)
 * @param len Length in bytes
 * @param timeout_ms Timeout to wait for master transaction (ms)
 * @param out_trans_len Pointer to store actual transaction length (can be NULL)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t spi_slave_driver_transmit_receive(int host_id, const uint8_t *tx_data, uint8_t *rx_data, size_t len, int timeout_ms, size_t *out_trans_len);

/**
 * @brief Check if SPI slave is configured on specified host
 * 
 * @param host_id SPI host ID
 * @return true if slave is configured, false otherwise
 */
bool spi_slave_driver_is_configured(int host_id);

/**
 * @brief Deinitialize SPI slave driver completely
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t spi_slave_driver_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // SPI_SLAVE_DRIVER_H

