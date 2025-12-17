#ifndef SPI_MASTER_DRIVER_H
#define SPI_MASTER_DRIVER_H

#include "driver/spi_master.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SPI_HOST_COUNT 3           // ESP32 has SPI_HOST (SPI1), HSPI_HOST (SPI2), VSPI_HOST (SPI3)
#define MAX_DEVICES_PER_HOST 8     // Maximum number of devices per SPI host

// SPI clock speed limits (approximate, depends on SPI host and config)
#define SPI_MAX_CLOCK_SPEED_HZ     80000000  // 80 MHz max for SPI3
#define SPI_MIN_CLOCK_SPEED_HZ     1000      // 1 kHz min reasonable speed

/**
 * @brief SPI bus configuration structure (for master mode)
 * 
 * @note For DMA transfers:
 *       - Buffers must be in DRAM (not flash)
 *       - 4-byte alignment recommended for high speeds
 */
typedef struct {
    int host_id;                    ///< SPI host ID (SPI1_HOST, SPI2_HOST, or SPI3_HOST)
    int mosi_io_num;                ///< MOSI GPIO pin number
    int miso_io_num;                ///< MISO GPIO pin number
    int sclk_io_num;                ///< SCLK GPIO pin number
    int quadwp_io_num;               ///< Quad WP GPIO pin number (-1 if not used)
    int quadhd_io_num;               ///< Quad HD GPIO pin number (-1 if not used)
    int dma_chan;                   ///< DMA channel (0 = no DMA, 1 or 2 for DMA, SPI_DMA_CH_AUTO for auto)
    int max_transfer_sz;             ///< Maximum transfer size in bytes (e.g., 4096 for small, 65536+ for displays/SD cards)
    int queue_size;                 ///< Transaction queue size
} spi_master_bus_config_t;

/**
 * @brief SPI device configuration structure (for master mode)
 */
typedef struct {
    int host_id;                    ///< SPI host ID
    int cs_io_num;                  ///< CS GPIO pin number (any valid GPIO, or -1 for software CS)
    uint32_t clock_speed_hz;        ///< Device-specific clock speed in Hz (required, max ~80MHz)
    uint8_t mode;                   ///< Device-specific SPI mode (0-3, required)
    int cs_ena_pretrans;            ///< CS setup time before transaction
    int cs_ena_posttrans;           ///< CS hold time after transaction
    int input_delay_ns;              ///< Input delay in nanoseconds
    uint32_t flags;                 ///< SPI device flags
    int queue_size;                 ///< Device transaction queue size
} spi_master_device_config_t;

/**
 * @brief Device registry entry - avoids tying array indices to GPIO numbers
 */
typedef struct {
    int cs_pin;                     ///< CS GPIO pin number (-1 = unused slot)
    spi_device_handle_t handle;     ///< SPI device handle
    bool in_use;                    ///< Whether this slot is in use
} spi_master_dev_entry_t;

/**
 * @brief Initialize SPI master driver
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t spi_master_driver_init(void);

/**
 * @brief Configure SPI bus for master mode
 * 
 * @param config SPI bus configuration structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t spi_master_driver_config_bus(const spi_master_bus_config_t *config);

/**
 * @brief Add SPI device to bus (master mode)
 * 
 * @param config SPI device configuration structure
 * @return esp_err_t ESP_OK on success, device handle is cached internally
 */
esp_err_t spi_master_driver_add_device(const spi_master_device_config_t *config);

/**
 * @brief Remove SPI device from bus (master mode)
 * 
 * @param host_id SPI host ID
 * @param cs_io_num CS GPIO pin number
 * @return esp_err_t ESP_OK on success
 */
esp_err_t spi_master_driver_remove_device(int host_id, int cs_io_num);

/**
 * @brief Transmit data to SPI device (master mode)
 * 
 * @param host_id SPI host ID
 * @param cs_io_num CS GPIO pin number
 * @param tx_data Pointer to transmit data buffer (must be in DRAM for DMA)
 * @param tx_len Length of data to transmit
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success
 */
esp_err_t spi_master_driver_transmit(int host_id, int cs_io_num, const uint8_t *tx_data, size_t tx_len, int timeout_ms);

/**
 * @brief Receive data from SPI device (master mode)
 * 
 * @param host_id SPI host ID
 * @param cs_io_num CS GPIO pin number
 * @param rx_data Pointer to receive data buffer (must be in DRAM for DMA)
 * @param rx_len Length of data to receive
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success
 */
esp_err_t spi_master_driver_receive(int host_id, int cs_io_num, uint8_t *rx_data, size_t rx_len, int timeout_ms);

/**
 * @brief Transmit and receive data (full-duplex, master mode)
 * 
 * @param host_id SPI host ID
 * @param cs_io_num CS GPIO pin number
 * @param tx_data Pointer to transmit data buffer (can be NULL, must be in DRAM for DMA)
 * @param rx_data Pointer to receive data buffer (can be NULL, must be in DRAM for DMA)
 * @param len Length of data to transmit/receive
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success
 */
esp_err_t spi_master_driver_transmit_receive(int host_id, int cs_io_num, const uint8_t *tx_data, uint8_t *rx_data, size_t len, int timeout_ms);

/**
 * @brief Check if SPI master bus is configured
 * 
 * @param host_id SPI host ID
 * @return true if bus is configured, false otherwise
 */
bool spi_master_driver_is_bus_configured(int host_id);

/**
 * @brief Deinitialize SPI master driver
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t spi_master_driver_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // SPI_MASTER_DRIVER_H

